// This target has no ROS dependencies and never calls FCU/actuator services.
// It runs the exact transaction engine used by GuidedProfileClient.
#define CUADC_GUIDED_PROFILE_TRANSACTION_ONLY
#include "../src/guided_profile_client.hpp"
#include <iostream>
#include <stdexcept>

using cuadc_guided::FcuProfile;
using namespace cuadc_guided::detail;

static void check(bool condition, const char * text)
{if (!condition) {throw std::runtime_error(text);}}
static TransactionConfig enabled(bool parameters = false, int retries = 0)
{
  TransactionConfig c;
  c.enabled = true;
  c.param_switch_enabled = parameters;
  c.confirmed_firmware = "4.5.7";
  c.timeout_s = 2.0;
  c.retries = retries;
  c.allowlist = {"ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D"};
  return c;
}
static FcuProfile gains()
{return {"approach", 0.6, {"ATC_RAT_RLL_P", "ATC_RAT_RLL_I"}, {0.11, 0.12}};}
static Operation send(ProfileTransaction & t, OperationKind kind, double now)
{
  auto op = t.next(now, true, true);
  check(op.has_value(), "missing operation");
  check(op->kind == kind, "incorrect operation order");
  check(t.dispatched(*op, now), "dispatch refused");
  check(!t.next(now, true, true), "parallel operations dispatched");
  return *op;
}
static OperationResult success(double value = 0.0)
{
  OperationResult r;
  r.success = true;
  r.accepted = true;
  r.is_double = true;
  r.value = value;
  return r;
}
static void snapshot(ProfileTransaction & t)
{
  const auto pull = send(t, OperationKind::Pull, 0.1);
  t.complete(pull, success(), 0.2);
  const auto read = send(t, OperationKind::Snapshot, 0.3);
  auto result = success();
  result.snapshot = {0.07, 0.08};
  t.complete(read, result, 0.4);
}

int main()
{
  try {
    {
      ProfileTransaction t{TransactionConfig{}};
      t.request(gains());
      check(t.ready() && !t.busy() && !t.failed(), "disabled client not ready");
      check(!t.next(0.0, true, true), "disabled client issued operation");
    }
    {
      ProfileTransaction t{enabled()};
      t.request({"cruise", 2.0, {}, {}});
      check(!t.next(0.0, false, true), "authority gate bypassed");
      check(!t.next(0.1, true, false), "settled gate bypassed");
      const auto command = send(t, OperationKind::Command, 0.2);
      check(command.value == 2.0 && !t.ready(), "speed or early readiness incorrect");
      t.complete(command, success(), 0.3);
      check(t.ready(), "accepted speed command not ready");
      t.request({"cruise", 2.0, {}, {}});
      check(send(t, OperationKind::Command, 0.4).generation != command.generation,
        "same-name request cached instead of reapplying");
    }
    {
      auto c = enabled(true);
      c.confirmed_firmware = "4.5.x";
      ProfileTransaction t{c}; t.request(gains());
      check(t.failed() && !t.next(0, true, true), "non-exact firmware accepted");
      c.confirmed_firmware = "4.6.0";
      ProfileTransaction wrong{c}; wrong.request(gains());
      check(wrong.failed(), "unsupported firmware accepted");
    }
    {
      ProfileTransaction t{enabled(true)};
      t.request({"unsafe", 1.0, {"WPNAV_SPEED"}, {100.0}});
      check(t.failed(), "non-runtime gain accepted");
      ProfileTransaction duplicate{enabled(true)};
      duplicate.request({"unsafe", 1.0, {"ATC_RAT_RLL_P", "ATC_RAT_RLL_P"}, {0.1, 0.1}});
      check(duplicate.failed(), "duplicate parameters accepted");
      auto c = enabled(true); c.allowlist.clear();
      ProfileTransaction deny{c}; deny.request(gains());
      check(deny.failed(), "empty allowlist accepted PID");
      ProfileTransaction disabled{enabled(false)}; disabled.request(gains());
      check(disabled.failed(), "PID silently ignored when switch disabled");
    }
    {
      ProfileTransaction t{enabled(true)}; t.request(gains()); snapshot(t);
      auto p = send(t, OperationKind::Write, 0.5);
      check(p.param_id == "ATC_RAT_RLL_P", "first gain incorrect");
      t.complete(p, success(p.value), 0.6);
      auto i = send(t, OperationKind::Write, 0.7);
      check(i.param_id == "ATC_RAT_RLL_I", "second gain incorrect");
      t.complete(i, success(i.value), 0.8);
      const auto speed = send(t, OperationKind::Command, 0.9);
      check(!t.ready(), "parameter ACKs bypassed speed ACK");
      t.complete(speed, success(), 1.0);
      check(t.ready(), "complete sequential transaction failed");
    }
    {
      ProfileTransaction t{enabled(true)}; t.request(gains());
      auto pull = send(t, OperationKind::Pull, 0.1);
      t.complete(pull, success(), 0.2);
      auto read = send(t, OperationKind::Snapshot, 0.3);
      auto bad = success(); bad.snapshot = {0.07};
      t.complete(read, bad, 0.4);
      check(t.failed() && !t.next(0.5, true, true), "missing snapshot allowed write");
    }
    {
      // A partial success plus ACK mismatch must restore BOTH touched values.
      ProfileTransaction t{enabled(true)}; t.request(gains()); snapshot(t);
      auto p = send(t, OperationKind::Write, 0.5); t.complete(p, success(p.value), 0.6);
      auto i = send(t, OperationKind::Write, 0.7); t.complete(i, success(9.0), 0.8);
      auto restore_p = send(t, OperationKind::Rollback, 0.9);
      check(restore_p.param_id == p.param_id && restore_p.value == 0.07, "P restore incorrect");
      t.complete(restore_p, success(restore_p.value), 1.0);
      auto restore_i = send(t, OperationKind::Rollback, 1.1);
      check(restore_i.param_id == i.param_id && restore_i.value == 0.08, "mismatched I not restored");
      t.complete(restore_i, success(restore_i.value), 1.2);
      check(t.failed() && !t.ready() && !t.busy(), "rollback resumed mission");
      t.request({"resume", 1.0, {}, {}});
      check(t.failed() && !t.next(1.3, true, true), "failure latch reset by new request");
    }
    {
      // A failed restoration does not skip later potentially changed values.
      ProfileTransaction t{enabled(true)}; t.request(gains()); snapshot(t);
      auto p = send(t, OperationKind::Write, 0.5); t.complete(p, success(p.value), 0.6);
      auto i = send(t, OperationKind::Write, 0.7); t.complete(i, success(9.0), 0.8);
      auto restore_p = send(t, OperationKind::Rollback, 0.9);
      t.complete(restore_p, success(99.0), 1.0);
      auto restore_i = send(t, OperationKind::Rollback, 1.1);
      t.complete(restore_i, success(restore_i.value), 1.2);
      check(t.failed() && t.status().find("rollback incomplete") != std::string::npos,
        "rollback failure not reported");
    }
    {
      ProfileTransaction t{enabled(true, 1)}; t.request(gains()); snapshot(t);
      auto first = send(t, OperationKind::Write, 0.5);
      t.complete(first, success(1.0), 0.6);
      auto retry = send(t, OperationKind::Write, 0.7);
      check(retry.index == first.index && retry.sequence != first.sequence, "retry incorrectly advanced");
      t.complete(retry, success(1.0), 0.8);
      auto restore = send(t, OperationKind::Rollback, 0.9);
      check(restore.param_id == first.param_id && restore.value == 0.07, "retry did not restore original");
      t.complete(restore, success(restore.value), 1.0);
      check(t.failed(), "retry budget not bounded");
    }
    {
      // Unknown write timeout: no rollback can race the service's late send.
      ProfileTransaction t{enabled(true, 3)}; t.request(gains()); snapshot(t);
      auto late = send(t, OperationKind::Write, 0.5);
      t.observe(2.5, true, true);
      check(t.failed() && t.uncertain() && !t.next(2.6, true, true), "timeout retried or restored");
      t.complete(late, success(late.value), 2.7);
      check(t.failed() && !t.ready(), "late ACK released failed transaction");
      t.request({"resume", 1.0, {}, {}});
      check(t.failed() && !t.next(2.8, true, true), "resume raced late mutation");
    }
    {
      ProfileTransaction t{enabled(true, 3)}; t.request(gains()); snapshot(t);
      auto write = send(t, OperationKind::Write, 0.5);
      OperationResult unknown; unknown.outcome_known = false;
      t.complete(write, unknown, 0.6);
      check(t.failed() && t.uncertain() && !t.next(0.7, true, true),
        "unqualified ParamSetV2 false retried or restored");
    }
    {
      ProfileTransaction t{enabled()}; t.request({"cruise", 1.0, {}, {}});
      auto command = send(t, OperationKind::Command, 0.1);
      t.observe(0.2, false, true);
      t.complete(command, success(), 0.3);
      check(t.failed() && t.uncertain() && !t.ready(), "pilot override ignored late ACK");
    }
    {
      ProfileTransaction t{enabled(true)}; t.request(gains());
      auto pull = send(t, OperationKind::Pull, 0.1);
      t.cancel("pilot override"); t.complete(pull, success(), 0.2);
      check(t.failed() && !t.uncertain() && !t.next(0.3, true, true), "cancelled read resumed writes");
    }
    {
      ProfileTransaction t{enabled(true)}; t.request(gains()); snapshot(t);
      auto write = send(t, OperationKind::Write, 0.5);
      t.observe(0.6, true, false); t.complete(write, success(write.value), 0.7);
      check(!t.next(0.8, true, false), "settled loss dispatched subsequent write");
      t.observe(2.8, true, false);
      check(t.failed() && !t.uncertain(), "paused operation did not time out safely");
    }
    {
      ProfileTransaction t{enabled()}; t.request({"cruise", 1.0, {}, {}});
      auto unavailable = t.next(0.0, true, true);
      check(unavailable.has_value(), "availability operation missing");
      t.observe(2.0, true, true);
      check(t.failed() && !t.uncertain(), "undispatched availability timeout marked mutation");
      t.complete(*unavailable, success(), 2.1);
      check(!t.ready(), "undispatched operation accepted completion");
    }
    {
      ProfileTransaction t{enabled()}; t.request({"cruise", 1.0, {}, {}});
      auto command = send(t, OperationKind::Command, 0.1);
      auto denied = success(); denied.accepted = false;
      t.complete(command, denied, 0.2);
      check(t.failed(), "CommandLong success without ACCEPTED passed gate");
    }
    std::cout << "guided profile transaction tests: 16 scenarios passed (no ROS or hardware)\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "guided profile transaction test failed: " << e.what() << '\n';
    return 1;
  }
}

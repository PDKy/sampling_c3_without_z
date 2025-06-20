/* clang-format off to disable clang-format-includes */
#include "solvers/fast_osqp_solver.h"
/* clang-format on */

#include "drake/common/never_destroyed.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/mathematical_program.h"

// This file contains implementations that are common to both the available and
// unavailable flavor of this class.

using drake::solvers::MathematicalProgram;
using drake::solvers::ProgramAttribute;
using drake::solvers::ProgramAttributes;
using drake::solvers::SolverId;

namespace dairlib {
namespace solvers {

FastOsqpSolver::FastOsqpSolver()
    : SolverBase(id(), &is_available, &is_enabled,
                 &ProgramAttributesSatisfied) {}

FastOsqpSolver::~FastOsqpSolver() = default;

SolverId FastOsqpSolver::id() {
  static const drake::never_destroyed<SolverId> singleton{"OSQP"};
  return singleton.access();
}

bool FastOsqpSolver::is_enabled() { return true; }

namespace {
bool CheckAttributes(const MathematicalProgram& prog,
                     std::string* explanation) {
  static const drake::never_destroyed<ProgramAttributes> solver_capabilities(
      std::initializer_list<ProgramAttribute>{
          ProgramAttribute::kLinearCost, ProgramAttribute::kQuadraticCost,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLinearEqualityConstraint});
  const ProgramAttributes& required_capabilities = prog.required_capabilities();
  const bool capabilities_match = AreRequiredAttributesSupported(
      required_capabilities, solver_capabilities.access(), explanation);
  if (!capabilities_match) {
    if (explanation) {
      *explanation = fmt::format("OsqpSolver is unable to solve because {}.",
                                 *explanation);
    }
    return false;
  }
  if (required_capabilities.count(ProgramAttribute::kQuadraticCost) == 0) {
    if (explanation) {
      *explanation =
          "OsqpSolver is unable to solve because a QuadraticCost is required"
          " but has not been declared; OSQP works best with a quadratic cost."
          " Please use a different solver such as CLP (for linear programming)"
          " or IPOPT/SNOPT (for nonlinear programming) if you don't want to add"
          " a quadratic cost to this program.";
    }
    return false;
  }
  const drake::solvers::Binding<drake::solvers::QuadraticCost>*
      nonconvex_quadratic_cost =
          drake::solvers::internal::FindNonconvexQuadraticCost(
              prog.quadratic_costs());
  if (nonconvex_quadratic_cost != nullptr) {
    if (explanation) {
      *explanation =
          "OsqpSolver is unable to solve because the quadratic cost " +
          nonconvex_quadratic_cost->to_string() +
          " is non-convex. Either change this cost to a convex one, or switch "
          "to a different solver like SNOPT/IPOPT/NLOPT.";
    }
    return false;
  }
  if (explanation) {
    explanation->clear();
  }
  return true;
}
}  // namespace

bool FastOsqpSolver::ProgramAttributesSatisfied(
    const MathematicalProgram& prog) {
  return CheckAttributes(prog, nullptr);
}

std::string FastOsqpSolver::UnsatisfiedProgramAttributes(
    const MathematicalProgram& prog) {
  std::string explanation;
  CheckAttributes(prog, &explanation);
  return explanation;
}

}  // namespace solvers
}  // namespace dairlib

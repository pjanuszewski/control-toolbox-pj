/*!
 * Simple example how to linearize a system and design an LQR controller.
 *
 * \example LQR.cpp
 */


#include <ct/optcon/optcon.h>  // also includes ct_core
#include "exampleDir.h"

// template <size_t STATE_DIM, size_t CONTROL_DIM>
// const typename LQR<STATE_DIM, CONTROL_DIM>::control_feedback_t& LQR<STATE_DIM, CONTROL_DIM>::get_K() const {
//     return K;
// }

int main(int argc, char** argv)
{
    // get the state and control input dimension of the oscillator
    // const size_t state_dim = ct::core::SecondOrderSystem::STATE_DIM;
    // const size_t control_dim = ct::core::SecondOrderSystem::CONTROL_DIM;

    // // create an auto-differentiable instance of the oscillator dynamics
    // ct::core::ADCGScalar w_n(50.0);
    // std::shared_ptr<ct::core::ControlledSystem<state_dim, control_dim, ct::core::ADCGScalar>> oscillatorDynamics(
    //     new ct::core::tpl::SecondOrderSystem<ct::core::ADCGScalar>(w_n));

    // // create an Auto-Differentiation Linearizer with code generation on the quadrotor model
    // ct::core::ADCodegenLinearizer<state_dim, control_dim> adLinearizer(oscillatorDynamics);

    // // compile the linearized model just-in-time
    // adLinearizer.compileJIT();

    // // define the linearization point around steady state
    // ct::core::StateVector<state_dim> x;
    // x.setZero();
    // ct::core::ControlVector<control_dim> u;
    // u.setZero();
    // double t = 0.0;

    // // compute the linearization around the nominal state using the Auto-Diff Linearizer
    // auto A = adLinearizer.getDerivativeState(x, u, t);
    // auto B = adLinearizer.getDerivativeControl(x, u, t);

    // // load the weighting matrices
    // ct::optcon::TermQuadratic<state_dim, control_dim> quadraticCost;
    // quadraticCost.loadConfigFile(ct::optcon::exampleDir + "/lqrCost.info", "termLQR");
    // auto Q = quadraticCost.stateSecondDerivative(x, u, t);    // x, u and t can be arbitrary here
    // auto R = quadraticCost.controlSecondDerivative(x, u, t);  // x, u and t can be arbitrary here

    static const size_t stateDim = 6;
    static const size_t controlDim = 4;

	// static constexpr float A_z = 0.1f;
	static constexpr float A_r = 0.2f;
	static constexpr float I_xx = 0.02f; // Inertia around the X-axis
	static constexpr float I_yy = 0.02f; // Inertia around the Y-axis
	static constexpr float I_zz = 0.04f; // Inertia around the Z-axis
	// static constexpr float mass = 1.5f;  // Mass of the vehicle

    Eigen::Matrix<double, stateDim, stateDim> A;
	Eigen::Matrix<double, stateDim, controlDim> B;
	Eigen::Matrix<double, stateDim, stateDim> Q;
	Eigen::Matrix<double, controlDim, controlDim> R;

    A << 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1,
    0, 0, 0, -A_r / I_xx, 0, 0,
    0, 0, 0, 0, -A_r / I_yy, 0,
    0, 0, 0, 0, 0, -A_r / I_zz;

    B << 0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 1/I_xx, 0, 0,
    0, 0, 1/I_yy, 0,
    0, 0, 0, 1/I_zz;

    Q << 0.5, 0, 0, 0, 0, 0,
    0, 0.5, 0, 0, 0, 0,
    0, 0, 0.5, 0, 0, 0,
    0, 0, 0, 0.5, 0, 0,
    0, 0, 0, 0, 0.5, 0,
    0, 0, 0, 0, 0, 0.5;

    R << 0.5, 0, 0, 0,
    0, 0.5, 0, 0,
    0, 0, 0.5, 0,
    0, 0, 0, 0.5;

    // design the LQR controller
    ct::optcon::LQR<stateDim, controlDim> lqrSolver;
    ct::optcon::LQR<stateDim, controlDim>::control_feedback_t K;

    std::cout << "A: " << std::endl << A << std::endl << std::endl;
    std::cout << "B: " << std::endl << B << std::endl << std::endl;
    std::cout << "Q: " << std::endl << Q << std::endl << std::endl;
    std::cout << "R: " << std::endl << R << std::endl << std::endl;

    lqrSolver.compute(Q, R, A, B, K);

    std::cout << "LQR gain matrix:" << std::endl << K << std::endl;

    std::ofstream outputFile("/home/pawelj/Git_repos/PX4-Autopilot/src/lib/rate_control/K_value.txt");
    outputFile << "LQR gain matrix:" << std::endl << K << std::endl;
    outputFile.close();

    return 0;
}

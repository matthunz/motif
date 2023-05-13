from time import time
import numpy as np
import motulator as mt
from maturin_starter import ExampleClass

base = mt.BaseValues(
    U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6, P_nom=2.2e3, n_p=2)


class Control(mt.InductionMotorVHzCtrl):
    def __init__(self):
        self.controller = ExampleClass();
        super().__init__(mt.InductionMotorVHzCtrlPars(R_s=0, R_R=0, k_u=0, k_w=0))

    def __call__(self, mdl):
        a, b = self.controller.control(mdl.motor.meas_currents(), mdl.conv.meas_dc_voltage(), 0, mdl.t0)
        c, d =  super().__call__(mdl)

        return [c, b]


def L_s(psi):
    """
    Stator inductance saturation model for a 2.2-kW motor.

    Parameters
    ----------
    psi : float
        Magnitude of the stator flux linkage (Vs).
    
    Returns
    -------
    float
        Stator inductance (H).

    """
    # Saturation model parameters for a 2.2-kW induction motor
    L_su, beta, S = .34, .84, 7
    # Stator inductance
    return L_su/(1 + (beta*psi)**S)


# %%
# Create the system model.

mdl = mt.InductionMotorDriveDiode()
# Γ-equivalent motor model with main-flux saturation included
mdl.motor = mt.InductionMotorSaturated(
    n_p=2, R_s=3.7, R_r=2.5, L_ell=.023)
# Mechanics model
mdl.mech = mt.Mechanics(J=.015)
# Frequency converter with a diode bridge
mdl.conv = mt.FrequencyConverter(L=2e-3, C=235e-6, U_g=400, f_g=50)

# %%
# Control system (parametrized as open-loop V/Hz control).

ctrl = Control()

# %%
# Set the speed reference and the external load torque. More complicated
# signals could be defined as functions.

ctrl.w_m_ref = lambda t: (t > .2)*(1.*base.w)

# Quadratic load torque profile (corresponding to pumps and fans)
k = 1.1*base.tau_nom/(base.w/base.n_p)**2
mdl.mech.tau_L_w = lambda w_M: k*w_M**2*np.sign(w_M)

# Stepwise load torque at t = 1 s, 20% of the rated torque
mdl.mech.tau_L_t = lambda t: (t > 1.)*base.tau_nom*.2

# %%
# Create the simulation object and simulate it. The option `pwm=True` enables
# the model for the carrier comparison.

sim = mt.Simulation(mdl, ctrl, pwm=True)
t_start = time()  # Start the timer
sim.simulate(t_stop=1.5)
print(f'\nExecution time: {(time() - t_start):.2f} s')

mt.plot(sim, base=base)
mt.plot_extra(sim, t_span=(1.1, 1.125), base=base)

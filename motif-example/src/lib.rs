use motif::induction::InductionMotorVhzControl;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use pyo3::wrap_pymodule;

mod submodule;

#[pyclass]
struct ExampleClass {
    control: InductionMotorVhzControl,
}

#[pymethods]
impl ExampleClass {
    #[new]
    pub fn new() -> Self {
        let mut control = InductionMotorVhzControl::builder().r_r(0.).build();
        control.r_s = 0.;
        control.k_u = 0.;
        control.k_w = Default::default();
        ExampleClass { control }
    }

    pub fn control(
        &mut self,
        i_s_abc: [f32; 3],
        u_dc: f32,
        w_m_ref: f32,
        t: f32,
    ) -> (f32, [f32; 3]) {
        self.control.control(i_s_abc, u_dc, w_m_ref, t)
    }
}

/// An example module implemented in Rust using PyO3.
#[pymodule]
fn maturin_starter(py: Python<'_>, m: &PyModule) -> PyResult<()> {
    m.add_class::<ExampleClass>()?;
    m.add_wrapped(wrap_pymodule!(submodule::submodule))?;

    // Inserting to sys.modules allows importing submodules nicely from Python
    // e.g. from maturin_starter.submodule import SubmoduleClass

    let sys = PyModule::import(py, "sys")?;
    let sys_modules: &PyDict = sys.getattr("modules")?.downcast()?;
    sys_modules.set_item("maturin_starter.submodule", m.getattr("submodule")?)?;

    Ok(())
}

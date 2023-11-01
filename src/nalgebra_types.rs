pub use nalgebra::base::dimension::*;
pub use nalgebra::base::*;
pub use nalgebra::*;
pub use num_traits::float::Float;
pub use num_traits::identities::{One, Zero};

// So there is from_f32 somehow for RealField
// But it returns an optional.
// So this gives us as_* and from_* easily.
// We need those for reading and writing files.
pub trait FloatField: RealField + Copy{
    fn half() -> Self;
    fn as_f32(self) -> f32;
    fn frm_f32(x: f32) -> Self;
    fn as_f64(self) -> f64;
    fn frm_f64(x: f64) -> Self;
}

impl FloatField for f32 {
    fn half() -> f32 {
        0.5f32
    }

    fn as_f32(self) -> f32 {
        self
    }

    fn frm_f32(x: f32) -> f32 {
        x
    }

    fn as_f64(self) -> f64 {
        self as f64
    }

    fn frm_f64(x: f64) -> f32 {
        x as f32
    }
}

impl FloatField for f64 {
    fn half() -> f64 {
        0.5
    }

    fn as_f32(self) -> f32 {
        self as f32
    }

    fn frm_f32(x: f32) -> f64 {
        x as f64
    }

    fn as_f64(self) -> f64 {
        self
    }

    fn frm_f64(x: f64) -> f64 {
        x
    }
}

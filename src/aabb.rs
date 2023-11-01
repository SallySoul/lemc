use crate::nalgebra_types::*;

/// TODO
pub struct AABB<T: RealField, const D: usize> {
    min: SVector<T, D>,
    max: SVector<T, D>,
}

impl<T: RealField, const D: usize> AABB<T, D> {
    /// TODO
    pub fn from_points(min: SVector<T, D>, max: SVector<T, D>) -> AABB<T, D> {
        AABB { min, max }
    }

    pub fn from_point(c: SVector<T, D>) -> AABB<T, D> {
        AABB { min: c.clone(), max: c}
    }

    /// TODO
    pub fn blank() -> AABB<T, D> {
        AABB {
            min: SVector::<T, D>::from_element(T::max_value().unwrap()),
            max: SVector::<T, D>::from_element(T::min_value().unwrap()),
        }
    }

    /// TODO
    pub fn mut_add_point(&mut self, p: &SVector<T, D>) {
        self.min = self.min.inf(p);
        self.max = self.max.sup(p);
    }

    /// TODO
    pub fn mut_add_aabb(&mut self, other: &AABB<T, D>) {
        self.min = self.min.inf(&other.min);
        self.max = self.max.sup(&other.max);
    }

    /// TODO
    pub fn contains_point(&self, p: &SVector<T, D>) -> bool {
        p >= &self.min && p <= &self.max
    }

    /// TODO
    pub fn contains_aabb(&self, other: &AABB<T, D>) -> bool {
        other.min >= self.min && other.max <= self.max
    }

    /// TODO
    pub fn unite<'a, I: Iterator<Item = &'a AABB<T, D>>>(iter: I) -> AABB<T, D> {
        iter.fold(AABB::blank(), |mut a, b| {
            a.mut_add_aabb(b);
            a
        })
    }

    /// TODO
    pub fn measure(&self) -> T {
        (&self.max - &self.min).product()
    }

    /// TODO
    pub fn centroid(&self) -> SVector<T, D> {
        let dif: SVector<T, D> = (&self.max - &self.min) * (T::from_f32(0.5).unwrap());
        let result: SVector<T, D> = &self.min + dif;
        result
    }

    /// Return the size of the Box.
    pub fn diagonal(&self) -> SVector<T, D> {
        &self.max - &self.min
    }
}

// In general this can be implemented with a circulant matrix.
// Instead I hand write 2 and 3 dimensional implementations.
pub trait BoundaryMeasure<T: Scalar> {
    fn boundary_measure(&self) -> T;
}

impl<T: RealField> BoundaryMeasure<T> for AABB<T, 2> {
    fn boundary_measure(&self) -> T {
        let lengths: SVector<T, 2> = &self.min - &self.max;
        (one::<T>() + one::<T>()) * lengths.sum()
    }
}

impl<T: RealField + Copy> BoundaryMeasure<T> for AABB<T, 3> {
    fn boundary_measure(&self) -> T {
        let lengths: SVector<T, 3> = self.max - self.min;

        #[rustfmt::skip]
        let half =
            lengths[0] * lengths[1]
            + lengths[1] * lengths[2]
            + lengths[2] * lengths[0];
        (one::<T>() + one::<T>()) * half
    }
}

#[cfg(test)]
mod unit_tests {
    use super::*;

    #[test]
    fn contains_point_d1_f32() {
        let lower_bound = vector![0.0f32];
        let upper_bound = vector![1.0f32];
        let mut aabb = AABB::from_points(lower_bound, lower_bound);
        assert!(aabb.contains_point(&lower_bound));
        assert!(!aabb.contains_point(&upper_bound));

        aabb.mut_add_point(&upper_bound);
        assert!(aabb.contains_point(&lower_bound));
        assert!(aabb.contains_point(&upper_bound));

        let mut test_p = vector![1.0f32];
        for x in &[-1.0f32, 2.0, -2.355, 5.0] {
            test_p[0] = *x;
            assert!(!aabb.contains_point(&test_p));
        }
        for x in &[0.02, 0.523, 0.999999, 0.00000002] {
            test_p[0] = *x;
            assert!(aabb.contains_point(&test_p));
        }
    }

    #[test]
    fn contains_aabb_d2_f64() {
        let unit = AABB::from_points(vector![0.0f64, 0.0], vector![1.0, 1.0]);
        let contained = AABB::from_points(vector![0.1, 0.4], vector![0.3, 0.8]);
        assert!(unit.contains_aabb(&contained));

        let outer = AABB::from_points(vector![-0.1, -0.1], vector![2.1, 2.0]);
        assert!(!unit.contains_aabb(&outer));

        let partial = AABB::from_points(vector![0.5, -1.0], vector![1.0, 2.0]);
        assert!(!unit.contains_aabb(&partial));
    }

    #[test]
    fn unite_d2_f64() {
        let aabbs = vec![
            AABB::from_points(vector![0.0, 0.0], vector![1.0, 1.0]),
            AABB::from_points(vector![1.0, 1.0], vector![2.0, 2.0]),
            AABB::from_points(vector![-2.0, -2.0], vector![-1.0, -1.0]),
        ];

        for a in &aabbs {
            assert!(a.measure() == 1.0);
        }

        let r = AABB::unite(aabbs.iter());
        assert!(r.contains_point(&vector![0.5, 2.0]));
        assert!(!r.contains_point(&vector![2.6, 0.01]));
    }
}

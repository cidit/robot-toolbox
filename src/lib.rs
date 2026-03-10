#![no_std]

pub trait PID {
    fn compute(&mut self) -> Option<f32>;
    /** * @brief resets the integral term */
    fn reset(&mut self);
}

pub mod stats {

    #[derive(Copy, Clone)]
    pub struct Magnitude(f32);

    impl Magnitude {
        pub fn new(v: f32) -> Option<Magnitude> {
            if v < 0.0 || v > 1.0 {
                None
            } else {
                Some(Magnitude(v))
            }
        }
    }

    /// https://en.wikipedia.org/wiki/Exponential_smoothing
    pub trait ExponentialMovingAverage {
        type Output;
        fn new(alpha: Magnitude) -> Self;
        fn alpha(&self) -> Magnitude;
        fn add(&mut self, v: Self::Output);
        fn calc(&self) -> Option<Self::Output>;
        /// {dt} can be any time unit. output is the same.
        // TODO: refactor with some sort of `Duration`? (crate)
        fn time_constant(&self, dt: f32) -> f32 {
            self.alpha().0 / dt
        }
    }

    pub struct FloatingPointEMA {
        alpha: Magnitude,
        last: f32,
        init: bool,
    }

    impl ExponentialMovingAverage for FloatingPointEMA {
        type Output = f32;
        fn new(alpha: Magnitude) -> Self {
            return Self {
                alpha,
                last: 0.0,
                init: false,
            };
        }

        fn add(&mut self, v: f32) {
            if !self.init {
                self.last = v;
                self.init = true;
            } else {
                self.last = self.alpha.0 * v + (1.0 - self.alpha.0) * self.last;
            }
        }

        fn calc(&self) -> Option<f32> {
            if self.init { Some(self.last) } else { None }
        }

        fn alpha(&self) -> Magnitude {
            self.alpha
        }
    }
}

pub mod angles {

    use core::marker::PhantomData;

    pub trait Unit: Copy + Clone {
        fn half() -> f32;

        fn convert<OU: Unit>(value: f32) -> f32 {
            value / OU::half() * Self::half()
        }
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Degrees;
    impl Unit for Degrees {
        fn half() -> f32 {
            180.0
        }
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Radians;
    impl Unit for Radians {
        fn half() -> f32 {
            core::f32::consts::PI
        }
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Turns;
    impl Unit for Turns {
        fn half() -> f32 {
            0.5
        }
    }

    pub trait Domain: Copy + Clone {
        fn max<U: Unit>() -> f32;
        fn min<U: Unit>() -> f32;
        fn normalize<U: Unit>(value: f32) -> f32 {
            let centered_value = value - Self::min::<U>();
            let offset_max = Self::max::<U>() - Self::min::<U>();
            // homebrewed fmod
            let amount_fit = centered_value / offset_max;
            let amount_fit = libm::floorf(amount_fit);
            let normalized = centered_value - amount_fit * offset_max;
            return normalized + Self::min::<U>();
        }
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Principal;
    impl Domain for Principal {
        fn max<U: Unit>() -> f32 {
            U::half() * 2.0
        }

        fn min<U: Unit>() -> f32 {
            0.0
        }
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Centered;
    impl Domain for Centered {
        fn max<U: Unit>() -> f32 {
            U::half()
        }

        fn min<U: Unit>() -> f32 {
            -U::half()
        }
    }

    /**
     * @brief An angle that is expected to be winding. to get the unwound version, use `.normalize()`.
     */
    pub struct Angle<D: Domain, U: Unit> {
        value: f32,
        type_metadata: PhantomData<(D, U)>,
    }

    pub fn radians(value: f32) -> Angle<Principal, Radians> {
        Angle::<Principal, Radians> {
            value,
            type_metadata: PhantomData,
        }
    }
    pub fn degrees(value: f32) -> Angle<Principal, Degrees> {
        Angle::<Principal, Degrees> {
            value,
            type_metadata: PhantomData,
        }
    }
    pub fn turns(value: f32) -> Angle<Principal, Turns> {
        Angle::<Principal, Turns> {
            value,
            type_metadata: PhantomData,
        }
    }

    impl<D, U> Angle<D, U>
    where
        D: Domain,
        U: Unit,
    {
        pub fn parts(&self) -> (f32, f32) {
            (libm::cosf(self.value), libm::sinf(self.value))
        }

        pub fn val(&self) -> f32 {
            self.value
        }

        pub fn max() -> f32 {
            D::max::<U>()
        }

        pub fn min() -> f32 {
            D::min::<U>()
        }

        pub fn normalize(&self) -> Self {
            Self {
                value: D::normalize::<U>(self.value),
                type_metadata: PhantomData,
            }
        }

        pub fn travel(&self, destination: &Self) -> Angle<Centered, U> {
            Angle::<Centered, U> {
                value: destination.value - self.value,
                type_metadata: PhantomData,
            }
        }

        pub fn redefine<TD: Domain>(&self) -> Angle<TD, U> {
            // i realized that the only thing that changes when
            // changing domain is the normalization behaviour,
            // so this is just a helper to translate the types.
            Angle::<TD, U> {
                value: self.value,
                type_metadata: PhantomData,
            }
        }

        pub fn convert<TU: Unit>(&self) -> Angle<D, TU> {
            Angle::<D, TU> {
                value: U::convert::<TU>(self.value),
                type_metadata: PhantomData,
            }
        }
    }

    impl<D: Domain, U: Unit> From<f32> for Angle<D, U> {
        fn from(value: f32) -> Self {
            Self {
                value,
                type_metadata: PhantomData,
            }
        }
    }

    pub struct AngleMovingAverage {
        // alpha: f32,
        // avg_x: f32,
        // avg_y: f32,
        // init: bool,
        x: crate::stats::FloatingPointEMA,
        y: crate::stats::FloatingPointEMA,
    }

    impl crate::stats::ExponentialMovingAverage for AngleMovingAverage {
        type Output = Angle<Centered, Radians>;
        fn new(alpha: crate::stats::Magnitude) -> Self {
            Self {
                x: crate::stats::FloatingPointEMA::new(alpha),
                y: crate::stats::FloatingPointEMA::new(alpha),
            }
        }

        fn add(&mut self, v: Self::Output) {
            // FIXME: currently always normalizes input, but shouldnt it not for winding angles?
            let (x, y) = v.normalize().parts();
            self.x.add(x);
            self.y.add(y);
        }

        fn calc(&self) -> Option<Self::Output> {
            let x = self.x.calc()?;
            let y = self.y.calc()?;

            Some(radians(libm::atan2f(y, x)).redefine())
        }

        fn alpha(&self) -> crate::stats::Magnitude {
            self.x.alpha()
        }
    }
}

#[cfg(test)]
mod tests {
    use angles::{Degrees, Domain, Principal};

    use super::*;

    #[test]
    fn it_works() {
        let result = 4;
        assert_eq!(result, 4);
    }

    #[test]
    fn normalizing_angle_below_min() {
        assert_eq!(179.0, Principal::normalize::<Degrees>(-181.0))
    }
}

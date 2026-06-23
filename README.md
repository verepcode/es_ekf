# es_ekf — IMU + GNSS State Estimation

A sensor-fusion library that fuses IMU and GNSS measurements to estimate
3D position, velocity, and orientation. The core filters are the **EKF** and
the **Error-State EKF (ES-EKF)**; the architecture is designed so that new
filters (UKF, InEKF, …) can be added with minimal duplication.

> This document describes the **target architecture**. Some files are still
> being migrated to this structure (see [Known Issues / TODO](#known-issues--todo)).

---

## 1. Architecture Overview

Every recursive Bayesian filter shares the same rhythm: **predict → update**.
Our EKF and ES-EKF share almost everything; they differ in **exactly one step**:
how the computed correction (`delta`) is applied to the state.

This observation points directly at the **Template Method** pattern: the
invariant algorithm skeleton lives in the base class, and the one varying step
lives in the derived classes.

```
FilterBase                (pure interface: predict / update / getState / getCovariance)
   └── KalmanFilterBase   (shared: full predict, update SKELETON, shared members)
          │                 ▲ single abstract hook: correctState(predicted, delta)
          ├── EKF          (correctState → direct/Euler correction)
          └── ESEKF        (correctState → error-state injection, multiplicative orientation)
```

### Where are the filters the same vs. different?

| Step | EKF | ES-EKF | Where it lives |
|---|:---:|:---:|---|
| State propagation (motion model) | ✓ | ✓ | `KalmanFilterBase::predict` |
| F, L Jacobians + Q noise | ✓ | ✓ | `KalmanFilterBase::predict` |
| Covariance propagation `FPFᵀ + LQLᵀ` | ✓ | ✓ | `KalmanFilterBase::predict` |
| Kalman gain `K = PHᵀ(HPHᵀ+R)⁻¹` | ✓ | ✓ | `KalmanFilterBase::update` |
| Covariance update `(I−KH)P` | ✓ | ✓ | `KalmanFilterBase::update` |
| **Applying the correction to the state** | direct / Euler addition | error-state injection (multiplicative) | `correctState` (**different**) |

So `KalmanFilterBase::update` computes `delta` and delegates the final step to
the abstract `correctState`:

```cpp
State KalmanFilterBase::update(const MeasurementData& meas, State& state_check) {
    // ... compute R, K, delta ...
    State state_hat = correctState(state_check, delta);   // ← THE ONE DIFFERING STEP
    // ... update covariance, store state_ ...
    return state_hat;
}
```

```cpp
// ESEKF: error-state logic lives only here
State ESEKF::correctState(const State& predicted, const Vec9& delta) {
    State s = predicted;
    s.applyErrorState(delta);   // injects orientation multiplicatively
    return s;
}

// EKF: direct/Euler correction (no error state)
State EKF::correctState(const State& predicted, const Vec9& delta) {
    State s = predicted;
    s.position += delta.segment<3>(0);
    s.velocity += delta.segment<3>(3);
    // orientation error is added in Euler space
    return s;
}
```

---

## 2. Layers and Responsibilities

### `FilterBase` — pure interface
The contract every filter conforms to. Contains no implementation.

```cpp
virtual State            predict(const ImuData& imu, double dt)            = 0;
virtual State            update(const MeasurementData& meas, State& s)     = 0;
virtual const State&     getState()      const = 0;
virtual Eigen::MatrixXd  getCovariance() const = 0;
```

The mandatory `getCovariance()` implies this interface is implicitly for
**Gaussian (covariance-carrying) filters**. This determines which filters fit
cleanly (see [Extending](#4-extending--adding-a-new-filter)).

### `KalmanFilterBase` — the shared Kalman machinery
Holds everything the EKF family shares:

- **Members:** `state_` (the single persistent estimate), `h_jac_`, `l_jac_`,
  `f_jac_`, gravity, noise variances (`var_imu_f_`, `var_imu_w_`, `var_gnss_`),
  `motion_model_`, `sensors_`.
- **`predict` (full):** propagates the nominal state with the IMU, builds the
  Jacobians, propagates the covariance.
- **`update` (skeleton):** computes measurement noise `R`, gain `K`, and `delta`;
  delegates the correction to `correctState`; updates the covariance.
- **Abstract hook:** `virtual State correctState(const State& predicted, const Vec9& delta) = 0;`

### `EKF` / `ESEKF` — implement only the difference
Each contains a single method (`correctState`) and a constructor. They inherit
`predict`/`update`; they do not reimplement them.

---

## 3. Data Types and Helpers

### State (`core/State.hpp`)
Error-state dimension is **9** (position 3 + velocity 3 + orientation error 3).
Orientation is stored as a full quaternion (`Orientation`); the error is handled
with the minimal 3D representation.

```cpp
struct State {
    Eigen::Vector3d position, velocity;
    Orientation     orient_in_quat;
    Eigen::Matrix<double,9,9> covariance;
    void applyErrorState(const Eigen::VectorXd& delta);  // adds pos/vel, injects orientation multiplicatively
};
```

### Measurements (`core/Types.hpp`)
A polymorphic measurement hierarchy — `update` always takes the base type, while
different derived types flow in:

```
MeasurementData          { value, covariance, timestamp; virtual ~ }
   ├── GNSSData          (value = position, 3×3 covariance)
   └── CameraData        (feature_points, feature_ids)
```

> The filter takes the **base type** via `update(const MeasurementData&, …)`,
> which keeps the override signature fixed across all filters. Inside the body,
> data is accessed through base members (`value`, `covariance`); derived-specific
> fields, if needed, are reached via `dynamic_cast` (made possible by
> `MeasurementData`'s `virtual` destructor).

### Helper components
- `core/Orientation.hpp` — quaternion wrapper (axis-angle / Euler conversions, products, `toMatrix`).
- `MathUtils.hpp` — `skewSymmetric`, angle normalization, RPY Jacobian, etc.
- `models/MotionModel.hpp` — abstract motion model (`propagate`, `getJacobian`, `getNoiseCovariance`).
- `sensors/SensorBase.hpp` — abstract sensor (`getNoiseCovariance`).

---

## 4. Extending — Adding a New Filter

### Filters that fit the interface cleanly (Gaussian)
These can be added under the same `KalmanFilterBase`; most only change *how* the
propagation/gain step is done:

| Filter | Difference from EKF |
|---|---|
| **UKF** | No Jacobians; captures nonlinearity via sigma points |
| **InEKF** | Error defined on a Lie group (ideal for orientation) |
| **IEKF** | Iterates the update until convergence |
| **Information Filter** | Works with the information matrix (P⁻¹) instead of covariance |

> Adding these may require widening the hook (e.g. making `predictCovariance`
> or `computeInnovation` virtual too).

### Filters that don't fit (future layering)
The `getCovariance()` / fixed `State` assumptions leave some filters out:

- **Particle Filter** — multi-modal distribution; a covariance is a poor fit.
- **Complementary / Mahony / Madgwick** — no covariance.
- **MSCKF** — needs a **variable-dimension** state (sliding window of camera
  poses), multi-pose constraints, and state augmentation; does not fit the fixed
  `State`/`9×9` assumption.

For these, the intended extension is to layer the interface:

```
Estimator                       (predict / update / getState)
 ├── GaussianFilter             (+ getCovariance)            → EKF, ES-EKF, UKF, InEKF
 └── SlidingWindowFilter        (+ augmentState / marginalize, dynamic state) → MSCKF
```

---

## 5. Directory Layout

```
include/
  core/        State.hpp, Types.hpp, Orientation.hpp
  filters/     FilterBase.hpp, KalmanFilterBase.hpp, EKF.hpp, ESEKF.hpp
  models/      MotionModel.hpp, ImuMotionModel.hpp
  sensors/     SensorBase.hpp, IMUSensor.hpp, GNSSSensor.hpp
  MathUtils.hpp, GPSUtils.hpp, StateEstimator.hpp
src/
  core/        EKF.cpp, ES_EKF.cpp, KalmanFilterBase.cpp
```

---

## 6. Typical Usage Flow

```cpp
State a_priori     = filter.predict(imu, dt);       // forward prediction with IMU
State a_posteriori = filter.update(gnss, a_priori); // correction with GNSS
// filter.getState() returns the current best estimate
```

When called through a `FilterBase*` (polymorphism), the runtime dispatches to the
correct filter (EKF or ES-EKF) — made possible by every filter overriding the
**identical signature**.

---

## 7. Known Issues / TODO

Open items to fully reach the architectural target:

- [x] Create `KalmanFilterBase` (`.hpp` + `.cpp`); move shared `predict`/`update`
      and members into it.
- [x] Slim down `EKF` and `ESEKF` to derive from `KalmanFilterBase` and implement
      only `correctState`.
- [x] Flatten header layout and remove the `es_ekf` namespace.
- [ ] Finish `ESEKF` header/source include wiring — it must include
      `filters/KalmanFilterBase.hpp` so `KalmanFilterBase`/`State` resolve;
      the filter sources do not yet fully compile.
- [ ] Reconcile the `ES_EKF.cpp` ↔ `ESEKF` (header) naming (consider renaming the
      source to `ESEKF.cpp`).
- [ ] Retire / adapt the legacy monolithic `StateEstimator` against the new layers.
```

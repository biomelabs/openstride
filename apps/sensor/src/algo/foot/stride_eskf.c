#include "algo/foot/stride_eskf.h"

#include <math.h>
#include <string.h>
#include <stddef.h>

/* ---- Timing / detection constants (match stride.c) ---- */
#define ESKF_GRAVITY_MS2                9.80665f
#define ESKF_MAX_SAMPLE_INTERVAL_US     1000000LL
#define ESKF_MIN_INTERVAL_US            400000LL
#define ESKF_MAX_INTERVAL_US            2000000LL
#define ESKF_MIN_STANCE_S               0.035f
#define ESKF_MIN_SWING_S                0.120f
#define ESKF_STANCE_GYRO_ENTER_RAD_S    1.4f
#define ESKF_STANCE_GYRO_EXIT_RAD_S     2.1f
#define ESKF_OUTPUT_ALPHA               0.4f
#define ESKF_MIN_LENGTH_M               0.20f
#define ESKF_MAX_LENGTH_M               3.20f
#define ESKF_DECAY_START_FALLBACK_US    450000LL
#define ESKF_STALE_DATA_FALLBACK_US     900000LL
#define ESKF_STALE_INTERVAL_SCALE       1.5f
#define ESKF_GYRO_BIAS_LIMIT_RAD_S      0.25f
#define ESKF_ACCEL_BIAS_LIMIT_MS2       2.0f

/* ---- ESKF tuning parameters ---- */
/*
 * Noise densities for ICM-42688-P class sensors.  All are 1-sigma values in
 * the continuous-time sense; they are converted to discrete process noise
 * covariance inside the prediction step via Q = σ² × dt.
 */
/* Gyro noise density [rad/s/√Hz] */
#define ESKF_SIGMA_GN    0.003f
/* Accel noise density [m/s²/√Hz] — ≈ 70 µg/√Hz */
#define ESKF_SIGMA_AN    (70.0e-6f * ESKF_GRAVITY_MS2)
/* Gyro bias random walk [rad/s²/√Hz] */
#define ESKF_SIGMA_GBW   1.0e-4f
/* Accel bias random walk [m/s³/√Hz] */
#define ESKF_SIGMA_ABW   5.0e-4f
/* ZUPT velocity measurement noise [m/s] — foot is nearly stationary in stance */
#define ESKF_SIGMA_ZUPT  0.01f

/* Initial diagonal covariance values */
#define ESKF_P0_THETA  (0.1f * 0.1f)
#define ESKF_P0_V      (0.5f * 0.5f)
#define ESKF_P0_P      (1.0f * 1.0f)
#define ESKF_P0_BG     (0.05f * 0.05f)
#define ESKF_P0_BA     (0.5f * 0.5f)

/* Error-state index base */
#define IX_TH  0   /* δθ : orientation error */
#define IX_V   3   /* δv : velocity error */
#define IX_P   6   /* δp : position error */
#define IX_BG  9   /* δb_g : gyro bias error */
#define IX_BA  12  /* δb_a : accel bias error */
#define N      STRIDE_ESKF_N

/* ---- Math helpers ---- */

static float eskf_absf(float v) { return v < 0.0f ? -v : v; }

static float eskf_clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static float eskf_dot3(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static float eskf_norm3(const float v[3])
{
    float d = eskf_dot3(v, v);
    return d > 0.0f ? sqrtf(d) : 0.0f;
}

static bool eskf_normalize3(float v[3])
{
    float n = eskf_norm3(v);
    if (n <= 1.0e-6f) return false;
    v[0] /= n; v[1] /= n; v[2] /= n;
    return true;
}

static void eskf_cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

static void eskf_quat_normalize(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n <= 1.0e-9f) { q[0]=1.0f; q[1]=q[2]=q[3]=0.0f; return; }
    q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n;
}

/* Rotate body→world using quaternion q [w,x,y,z] */
static void eskf_rotate_b2w(const float q[4], const float body[3], float world[3])
{
    float qw=q[0], qx=q[1], qy=q[2], qz=q[3];
    world[0] = (1.0f-2.0f*(qy*qy+qz*qz))*body[0] + 2.0f*(qx*qy-qw*qz)*body[1] + 2.0f*(qx*qz+qw*qy)*body[2];
    world[1] = 2.0f*(qx*qy+qw*qz)*body[0] + (1.0f-2.0f*(qx*qx+qz*qz))*body[1] + 2.0f*(qy*qz-qw*qx)*body[2];
    world[2] = 2.0f*(qx*qz-qw*qy)*body[0] + 2.0f*(qy*qz+qw*qx)*body[1] + (1.0f-2.0f*(qx*qx+qy*qy))*body[2];
}

/* Extract the rotation matrix R (body→world) from q */
static void eskf_quat_to_rotmat(const float q[4], float R[3][3])
{
    float qw=q[0], qx=q[1], qy=q[2], qz=q[3];
    R[0][0]=1.0f-2.0f*(qy*qy+qz*qz); R[0][1]=2.0f*(qx*qy-qw*qz); R[0][2]=2.0f*(qx*qz+qw*qy);
    R[1][0]=2.0f*(qx*qy+qw*qz);       R[1][1]=1.0f-2.0f*(qx*qx+qz*qz); R[1][2]=2.0f*(qy*qz-qw*qx);
    R[2][0]=2.0f*(qx*qz-qw*qy);       R[2][1]=2.0f*(qy*qz+qw*qx); R[2][2]=1.0f-2.0f*(qx*qx+qy*qy);
}

/* Skew-symmetric (cross-product) matrix of v */
static void eskf_skew3(const float v[3], float M[3][3])
{
    M[0][0]= 0.0f;  M[0][1]=-v[2]; M[0][2]= v[1];
    M[1][0]= v[2];  M[1][1]= 0.0f; M[1][2]=-v[0];
    M[2][0]=-v[1];  M[2][1]= v[0]; M[2][2]= 0.0f;
}

/* Invert a 3×3 matrix via cofactors; returns false if singular */
static bool eskf_mat3_inv(const float M[3][3], float inv[3][3])
{
    float det = M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
               -M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
               +M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
    if (eskf_absf(det) < 1.0e-20f) return false;
    float id = 1.0f / det;
    inv[0][0]= (M[1][1]*M[2][2]-M[1][2]*M[2][1])*id;
    inv[0][1]=-(M[0][1]*M[2][2]-M[0][2]*M[2][1])*id;
    inv[0][2]= (M[0][1]*M[1][2]-M[0][2]*M[1][1])*id;
    inv[1][0]=-(M[1][0]*M[2][2]-M[1][2]*M[2][0])*id;
    inv[1][1]= (M[0][0]*M[2][2]-M[0][2]*M[2][0])*id;
    inv[1][2]=-(M[0][0]*M[1][2]-M[0][2]*M[1][0])*id;
    inv[2][0]= (M[1][0]*M[2][1]-M[1][1]*M[2][0])*id;
    inv[2][1]=-(M[0][0]*M[2][1]-M[0][1]*M[2][0])*id;
    inv[2][2]= (M[0][0]*M[1][1]-M[0][1]*M[1][0])*id;
    return true;
}

/* q_out = q_a ⊗ q_b */
static void eskf_quat_mul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static void eskf_quat_from_two_vectors(const float from[3], const float to[3], float q[4])
{
    float dot = eskf_dot3(from, to);
    float axis[3];

    if (dot < -0.999f) {
        float ortho[3] = {1.0f, 0.0f, 0.0f};
        if (eskf_absf(from[0]) > 0.8f) { ortho[0]=0.0f; ortho[1]=1.0f; }
        eskf_cross3(from, ortho, axis);
        eskf_normalize3(axis);
        q[0]=0.0f; q[1]=axis[0]; q[2]=axis[1]; q[3]=axis[2];
        return;
    }
    eskf_cross3(from, to, axis);
    q[0] = 1.0f + dot; q[1]=axis[0]; q[2]=axis[1]; q[3]=axis[2];
    eskf_quat_normalize(q);
}

/* ---- Nominal state initialisation ---- */

static void eskf_orientation_init(stride_eskf_detector_t *d, const float accel[3])
{
    float up_body[3] = {accel[0], accel[1], accel[2]};
    const float up_world[3] = {0.0f, 0.0f, 1.0f};

    if (!eskf_normalize3(up_body)) {
        d->q[0]=1.0f; d->q[1]=d->q[2]=d->q[3]=0.0f;
    } else {
        eskf_quat_from_two_vectors(up_body, up_world, d->q);
    }
}

/* ---- Covariance prediction ---- */

/*
 * Propagate P via the linearised error-state dynamics:
 *   Ṗ = F·P + P·Fᵀ + Q_c
 * Discretised to first order: P += (F·P + P·Fᵀ)·dt + Q_d
 *
 * Sparse F non-zero blocks (continuous time):
 *   F[IX_TH, IX_TH] = -skew(ω_corr)
 *   F[IX_TH, IX_BG] = -I₃
 *   F[IX_V,  IX_TH] = -skew(a_corr_world)
 *   F[IX_V,  IX_BA] = -R
 *   F[IX_P,  IX_V]  =  I₃
 *
 * FP rows 9–14 are zero (bias error states have no autonomous dynamics),
 * so we only compute the first 9 rows.
 */
static void eskf_cov_predict(float P[N*N],
                             const float omega_corr[3],
                             const float a_corr_world[3],
                             const float R[3][3],
                             float dt)
{
    float Wx[3][3], Ax[3][3];
    eskf_skew3(omega_corr, Wx);
    eskf_skew3(a_corr_world, Ax);

    /* FP[i,j] for i=0..8 only; rows 9..14 are identically zero */
    static float FP[9 * N];

    for (int j = 0; j < N; j++) {
        /* Rows 0-2: F[IX_TH,IX_TH]·P[0:3,j] + F[IX_TH,IX_BG]·P[9:12,j]
         *         = -Wx·P[0:3,j] - I·P[9:12,j]                          */
        for (int i = 0; i < 3; i++) {
            float s = -P[(IX_BG + i)*N + j];
            for (int k = 0; k < 3; k++) s -= Wx[i][k] * P[k*N + j];
            FP[i*N + j] = s;
        }
        /* Rows 3-5: F[IX_V,IX_TH]·P[0:3,j] + F[IX_V,IX_BA]·P[12:15,j]
         *         = -Ax·P[0:3,j] - R·P[12:15,j]                         */
        for (int i = 0; i < 3; i++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s -= Ax[i][k] * P[k*N + j];
                s -= R[i][k]  * P[(IX_BA + k)*N + j];
            }
            FP[(IX_V + i)*N + j] = s;
        }
        /* Rows 6-8: F[IX_P,IX_V]·P[3:6,j] = P[3:6,j] */
        for (int i = 0; i < 3; i++) {
            FP[(IX_P + i)*N + j] = P[(IX_V + i)*N + j];
        }
    }

    /* P += (FP + FPᵀ)·dt  where FPᵀ[i,j] = FP[j,i] for j<9, else 0 */
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float fp_ij = (i < 9) ? FP[i*N + j] : 0.0f;
            float fp_ji = (j < 9) ? FP[j*N + i] : 0.0f;
            P[i*N + j] += (fp_ij + fp_ji) * dt;
        }
    }

    /* Diagonal process noise Q_d = σ²·dt per block */
    float q_th = ESKF_SIGMA_GN  * ESKF_SIGMA_GN  * dt;
    float q_v  = ESKF_SIGMA_AN  * ESKF_SIGMA_AN  * dt;
    float q_bg = ESKF_SIGMA_GBW * ESKF_SIGMA_GBW * dt;
    float q_ba = ESKF_SIGMA_ABW * ESKF_SIGMA_ABW * dt;
    for (int i = 0; i < 3; i++) {
        P[(IX_TH+i)*N+(IX_TH+i)] += q_th;
        P[(IX_V +i)*N+(IX_V +i)] += q_v;
        P[(IX_BG+i)*N+(IX_BG+i)] += q_bg;
        P[(IX_BA+i)*N+(IX_BA+i)] += q_ba;
    }
}

/* ---- ZUPT measurement update ---- */

/*
 * H = [0₃|I₃|0₉]  (selects δv)
 * z  = 0 − v_nominal
 * Rₘ = σ_zupt²·I₃
 *
 * Gain: K = P·Hᵀ·(H·P·Hᵀ + Rₘ)⁻¹
 * Joseph form: P = (I−KH)·P·(I−KH)ᵀ + K·Rₘ·Kᵀ  (preserves symmetry)
 */
static void eskf_zupt_update(stride_eskf_detector_t *d)
{
    const float Rm = ESKF_SIGMA_ZUPT * ESKF_SIGMA_ZUPT;

    /* S = P[IX_V:IX_V+3, IX_V:IX_V+3] + Rₘ·I₃ */
    float S[3][3], S_inv[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) S[i][j] = d->P[(IX_V+i)*N+(IX_V+j)];
        S[i][i] += Rm;
    }
    if (!eskf_mat3_inv(S, S_inv)) return;

    /* K = P[:,IX_V:IX_V+3]·S⁻¹   shape N×3 */
    float K[N][3];
    for (int i = 0; i < N; i++) {
        for (int m = 0; m < 3; m++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += d->P[i*N+(IX_V+k)] * S_inv[k][m];
            K[i][m] = s;
        }
    }

    /* Innovation z = 0 − v */
    float z[3] = {-d->v[0], -d->v[1], -d->v[2]};

    /* Error state correction δx = K·z */
    float dx[N];
    for (int i = 0; i < N; i++) dx[i] = K[i][0]*z[0] + K[i][1]*z[1] + K[i][2]*z[2];

    /* Inject δx into nominal state ---------------------------------------- */

    /* Orientation: q ← q ⊗ exp(δθ/2) ≈ q ⊗ [1, δθ/2] */
    float dq[4] = {1.0f, 0.5f*dx[IX_TH], 0.5f*dx[IX_TH+1], 0.5f*dx[IX_TH+2]};
    eskf_quat_normalize(dq);
    float q_new[4];
    eskf_quat_mul(d->q, dq, q_new);
    eskf_quat_normalize(q_new);
    d->q[0]=q_new[0]; d->q[1]=q_new[1]; d->q[2]=q_new[2]; d->q[3]=q_new[3];

    for (int i = 0; i < 3; i++) {
        d->v[i]   += dx[IX_V  + i];
        d->p[i]   += dx[IX_P  + i];
        d->b_g[i] += dx[IX_BG + i];
        d->b_a[i] += dx[IX_BA + i];
    }

    /* Clamp biases to physical limits */
    for (int i = 0; i < 3; i++) {
        d->b_g[i] = eskf_clampf(d->b_g[i], -ESKF_GYRO_BIAS_LIMIT_RAD_S,  ESKF_GYRO_BIAS_LIMIT_RAD_S);
        d->b_a[i] = eskf_clampf(d->b_a[i], -ESKF_ACCEL_BIAS_LIMIT_MS2,   ESKF_ACCEL_BIAS_LIMIT_MS2);
    }

    /* Update P (Joseph form) ------------------------------------------------
     * Step 1:  Pʼ = (I−KH)·P   →  Pʼ[i,j] = P[i,j] − Σₖ K[i,k]·P[IX_V+k, j]
     * Step 2:  P  = Pʼ·(I−KH)ᵀ + K·Rₘ·Kᵀ
     *             →  P[i,j] = Pʼ[i,j] − Σₖ Pʼ[i,IX_V+k]·K[j,k] + Rₘ·Σₖ K[i,k]·K[j,k]
     */
    static float Pp[N * N];

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float s = d->P[i*N + j];
            for (int k = 0; k < 3; k++) s -= K[i][k] * d->P[(IX_V+k)*N + j];
            Pp[i*N + j] = s;
        }
    }
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float s = Pp[i*N + j];
            for (int k = 0; k < 3; k++) {
                s -= Pp[i*N + (IX_V+k)] * K[j][k];
                s += Rm * K[i][k] * K[j][k];
            }
            d->P[i*N + j] = s;
        }
    }

    /* Enforce symmetry to prevent numerical drift */
    for (int i = 0; i < N; i++) {
        for (int j = i+1; j < N; j++) {
            float avg = 0.5f * (d->P[i*N+j] + d->P[j*N+i]);
            d->P[i*N+j] = avg;
            d->P[j*N+i] = avg;
        }
    }
}

/* ---- Stance detection ---- */

static bool eskf_is_stance_sample(const stride_eskf_detector_t *d,
                                  const float gyro[3])
{
    float gyro_norm = eskf_norm3(gyro);
    float band = d->in_stance ? ESKF_STANCE_GYRO_EXIT_RAD_S : ESKF_STANCE_GYRO_ENTER_RAD_S;
    return gyro_norm <= band;
}

/* ---- Stride output helpers (identical logic to stride.c) ---- */

static void eskf_decay_window_us(const stride_eskf_detector_t *d,
                                 int64_t *decay_start_us,
                                 int64_t *stale_interval_us)
{
    *decay_start_us   = ESKF_DECAY_START_FALLBACK_US;
    *stale_interval_us = ESKF_STALE_DATA_FALLBACK_US;

    if (d->last_stride_cadence_spm > 0.0f) {
        int64_t exp_us = (int64_t)((60.0f * 1000000.0f / d->last_stride_cadence_spm) + 0.5f);
        if (exp_us > *decay_start_us) *decay_start_us = exp_us;
        int64_t dyn_us = (int64_t)((float)exp_us * ESKF_STALE_INTERVAL_SCALE);
        if (dyn_us > *stale_interval_us) *stale_interval_us = dyn_us;
    }

    if (*stale_interval_us <= *decay_start_us) *stale_interval_us = *decay_start_us + 1;
}

/* Called at swing→stance transition with current timestamp */
static bool eskf_finalize_stride(stride_eskf_detector_t *d, int64_t timestamp_us)
{
    int64_t interval_us = timestamp_us - d->last_zupt_timestamp_us;

    if (d->last_zupt_timestamp_us < 0) {
        d->last_zupt_timestamp_us = timestamp_us;
        return false;
    }

    if ((interval_us < ESKF_MIN_INTERVAL_US) || (interval_us > ESKF_MAX_INTERVAL_US) ||
        (d->swing_elapsed_s < ESKF_MIN_SWING_S) || !d->p_swing_start_valid) {
        d->last_zupt_timestamp_us = timestamp_us;
        return false;
    }

    /* Horizontal (XY) displacement since the previous ZUPT */
    float dx = d->p[0] - d->p_swing_start[0];
    float dy = d->p[1] - d->p_swing_start[1];
    float stride_length_m = sqrtf(dx*dx + dy*dy);

    if (stride_length_m < ESKF_MIN_LENGTH_M) {
        d->last_zupt_timestamp_us = timestamp_us;
        return false;
    }
    if (stride_length_m > ESKF_MAX_LENGTH_M) stride_length_m = ESKF_MAX_LENGTH_M;

    float interval_s  = (float)interval_us * 1.0e-6f;
    float raw_cadence = 60.0f / interval_s;
    float raw_speed   = stride_length_m / interval_s;

    if (d->last_stride_cadence_spm > 0.0f) {
        d->last_stride_cadence_spm = ESKF_OUTPUT_ALPHA * raw_cadence
                                   + (1.0f - ESKF_OUTPUT_ALPHA) * d->last_stride_cadence_spm;
        d->last_stride_speed_mps   = ESKF_OUTPUT_ALPHA * raw_speed
                                   + (1.0f - ESKF_OUTPUT_ALPHA) * d->last_stride_speed_mps;
    } else {
        d->last_stride_cadence_spm = raw_cadence;
        d->last_stride_speed_mps   = raw_speed;
    }

    d->data.stride_count++;
    d->data.distance_m     += stride_length_m;
    d->data.stride_length_m = stride_length_m;
    d->data.cadence_spm     = d->last_stride_cadence_spm;
    d->data.speed_mps       = d->last_stride_speed_mps;
    d->data.update_latency_s = 0.0f;

    d->last_stride_timestamp_us = timestamp_us;
    d->last_zupt_timestamp_us   = timestamp_us;
    return true;
}

/* ---- Public API ---- */

void stride_eskf_detector_init(stride_eskf_detector_t *detector)
{
    if (detector == NULL) return;

    *detector = (stride_eskf_detector_t){
        .data              = sdm_data_zero(),
        .q                 = {1.0f, 0.0f, 0.0f, 0.0f},
        .v                 = {0.0f, 0.0f, 0.0f},
        .p                 = {0.0f, 0.0f, 0.0f},
        .b_g               = {0.0f, 0.0f, 0.0f},
        .b_a               = {0.0f, 0.0f, 0.0f},
        .stance_candidate_s = 0.0f,
        .in_stance         = false,
        .swing_elapsed_s   = 0.0f,
        .p_swing_start     = {0.0f, 0.0f, 0.0f},
        .p_swing_start_valid = false,
        .last_stride_speed_mps   = 0.0f,
        .last_stride_cadence_spm = 0.0f,
        .last_sample_timestamp_us = 0,
        .last_zupt_timestamp_us  = -1,
        .last_stride_timestamp_us = -1,
        .initialized       = false,
    };

    /* Initialise covariance with diagonal uncertainty */
    memset(detector->P, 0, sizeof(detector->P));
    for (int i = 0; i < 3; i++) {
        detector->P[(IX_TH+i)*N+(IX_TH+i)] = ESKF_P0_THETA;
        detector->P[(IX_V +i)*N+(IX_V +i)] = ESKF_P0_V;
        detector->P[(IX_P +i)*N+(IX_P +i)] = ESKF_P0_P;
        detector->P[(IX_BG+i)*N+(IX_BG+i)] = ESKF_P0_BG;
        detector->P[(IX_BA+i)*N+(IX_BA+i)] = ESKF_P0_BA;
    }
}

bool stride_eskf_detector_update(stride_eskf_detector_t *detector,
                                 const float accel_mps2[3],
                                 const float gyro_rads[3],
                                 int64_t timestamp_us,
                                 sdm_data_t *out_data)
{
    if ((detector == NULL) || (accel_mps2 == NULL) || (gyro_rads == NULL)) return false;

    if (!detector->initialized) {
        eskf_orientation_init(detector, accel_mps2);
        detector->last_sample_timestamp_us = timestamp_us;
        detector->initialized = true;
        detector->in_stance = eskf_is_stance_sample(detector, gyro_rads);
        detector->stance_candidate_s = detector->in_stance ? ESKF_MIN_STANCE_S : 0.0f;
        if (detector->in_stance) detector->last_zupt_timestamp_us = timestamp_us;
        if (out_data != NULL) *out_data = detector->data;
        return false;
    }

    int64_t dt_us = timestamp_us - detector->last_sample_timestamp_us;
    if (dt_us <= 0) {
        if (out_data != NULL) *out_data = detector->data;
        return false;
    }
    if (dt_us > ESKF_MAX_SAMPLE_INTERVAL_US) dt_us = ESKF_MAX_SAMPLE_INTERVAL_US;
    float dt = (float)dt_us * 1.0e-6f;

    /* ---- 1. Propagate nominal state ---- */

    float omega_corr[3] = {
        gyro_rads[0] - detector->b_g[0],
        gyro_rads[1] - detector->b_g[1],
        gyro_rads[2] - detector->b_g[2],
    };
    float a_corr_body[3] = {
        accel_mps2[0] - detector->b_a[0],
        accel_mps2[1] - detector->b_a[1],
        accel_mps2[2] - detector->b_a[2],
    };

    /* Integrate gyro into quaternion (first-order) */
    float hdt = 0.5f * dt;
    float *q = detector->q;
    float qw=q[0], qx=q[1], qy=q[2], qz=q[3];
    float wx=omega_corr[0], wy=omega_corr[1], wz=omega_corr[2];
    q[0] += hdt * (-qx*wx - qy*wy - qz*wz);
    q[1] += hdt * ( qw*wx + qy*wz - qz*wy);
    q[2] += hdt * ( qw*wy + qz*wx - qx*wz);
    q[3] += hdt * ( qw*wz + qx*wy - qy*wx);
    eskf_quat_normalize(q);

    /* Rotate bias-corrected accel to world frame */
    float a_corr_world[3];
    eskf_rotate_b2w(q, a_corr_body, a_corr_world);

    /* Linear acceleration (subtract gravity) in world frame */
    float lin_world[3] = {
        a_corr_world[0],
        a_corr_world[1],
        a_corr_world[2] - ESKF_GRAVITY_MS2,
    };

    /* Velocity and position Euler integration */
    float *v = detector->v;
    float *p = detector->p;
    for (int i = 0; i < 3; i++) {
        p[i] += v[i] * dt + 0.5f * lin_world[i] * dt * dt;
        v[i] += lin_world[i] * dt;
    }

    /* ---- 2. Propagate error covariance ---- */

    float R[3][3];
    eskf_quat_to_rotmat(q, R);
    eskf_cov_predict(detector->P, omega_corr, a_corr_world, R, dt);

    /* ---- 3. Stance detection ---- */

    bool stance_sample = eskf_is_stance_sample(detector, gyro_rads);
    if (stance_sample) {
        detector->stance_candidate_s += dt;
    } else {
        detector->stance_candidate_s = 0.0f;
    }

    bool in_stance_now = detector->in_stance
        ? stance_sample
        : (detector->stance_candidate_s >= ESKF_MIN_STANCE_S);

    /* ---- 4. Stance/swing state machine ---- */

    bool stride_detected = false;

    if (detector->in_stance && !in_stance_now) {
        /* Stance → swing: record foot-lift position for upcoming stride */
        detector->p_swing_start[0] = p[0];
        detector->p_swing_start[1] = p[1];
        detector->p_swing_start[2] = p[2];
        detector->p_swing_start_valid = true;
        detector->swing_elapsed_s = 0.0f;
    } else if (!detector->in_stance && in_stance_now) {
        /* Swing → stance: ZUPT first (corrects p before stride measurement),
         * then compute stride length from the corrected position. */
        eskf_zupt_update(detector);
        v[0] = v[1] = v[2] = 0.0f;
        stride_detected = eskf_finalize_stride(detector, timestamp_us);
    } else if (in_stance_now) {
        /* Continuous stance: keep applying ZUPT every sample. */
        eskf_zupt_update(detector);
        v[0] = v[1] = v[2] = 0.0f;
    } else {
        /* Swing: track elapsed time for minimum-swing guard. */
        detector->swing_elapsed_s += dt;
    }

    /* ---- 6. Output decay ---- */

    if (!stride_detected && (detector->last_stride_timestamp_us >= 0)) {
        int64_t latency_us = timestamp_us - detector->last_stride_timestamp_us;
        int64_t decay_start_us, stale_interval_us;

        if (latency_us < 0) latency_us = 0;
        eskf_decay_window_us(detector, &decay_start_us, &stale_interval_us);
        detector->data.update_latency_s = (float)latency_us * 1.0e-6f;

        if (latency_us >= stale_interval_us) {
            detector->data.cadence_spm = 0.0f;
            detector->data.speed_mps   = 0.0f;
        } else if (latency_us >= decay_start_us) {
            float fade = (float)(latency_us - decay_start_us)
                       / (float)(stale_interval_us - decay_start_us);
            fade = eskf_clampf(fade, 0.0f, 1.0f);
            detector->data.cadence_spm = detector->last_stride_cadence_spm * (1.0f - fade);
            detector->data.speed_mps   = detector->last_stride_speed_mps   * (1.0f - fade);
        }
    }

    detector->in_stance = in_stance_now;
    detector->last_sample_timestamp_us = timestamp_us;

    if (out_data != NULL) *out_data = detector->data;
    return stride_detected;
}

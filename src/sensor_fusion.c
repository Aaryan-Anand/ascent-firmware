#include "sensor_fusion.h"

#include "math.h"
#include "beep.h"

// Lazy-initialized function to provide initial reference vectors
const initial_vectors_t* get_initial_vectors() {
    static initial_vectors_t init_vectors;
    static bool initialized = false;

    if (!initialized) {
        imu_raw_3d_t acc, gyr, mag;
        float acc_sum;
        uint8_t acc_wait_count = 0;

        // Wait for stable gravity vector (~1g)
        while (1) {
            bno055_get_local(&acc, &gyr, &mag, false);
            acc_sum = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

            if (fabsf(acc_sum - 1000.0f) <= 100.0f) {
                printf("Accelerometer validated: %.1f mg\n", acc_sum);
                break;
            }

            error_beep();
            if (++acc_wait_count > 40) {
                printf("Warning: Accelerometer not stable, vector sum: %.1f mg\n", acc_sum);
                break;
            }
        }

        // Final reading with calibration ON
        bno055_get_local(&acc, &gyr, &mag, true);

        init_vectors.acc = acc;
        init_vectors.mag = mag;
        initialized = true;

        printf("Initial reference set.\n");
        printf("Gravity: [%.2d, %.2d, %.2d] mg\n", acc.x, acc.y, acc.z);
        printf("Magnetic: [%.2d, %.2d, %.2d] uT\n", mag.x, mag.y, mag.z);
    }

    return &init_vectors;
}

Orientation get_mag_orientation_with_reference(float mag_x, float mag_y, float mag_z) {
    Orientation o;

    const initial_vectors_t* init = get_initial_vectors();
    imu_raw_3d_t acc = init->acc;
    imu_raw_3d_t mag_ref = init->mag;

    // Normalize current magnetic field
    float mag_norm = sqrtf(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
    if (mag_norm == 0.0f) {
        o.roll = o.pitch = o.yaw = 0.0f;
        return o;
    }
    float m_x = mag_x / mag_norm;
    float m_y = mag_y / mag_norm;
    float m_z = mag_z / mag_norm;

    // Normalize reference gravity vector (Z axis)
    float g_norm = sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    float z_x = acc.x / g_norm;
    float z_y = acc.y / g_norm;
    float z_z = acc.z / g_norm;

    // Normalize reference magnetic field
    float n_norm = sqrtf(mag_ref.x * mag_ref.x + mag_ref.y * mag_ref.y + mag_ref.z * mag_ref.z);
    float n_x = mag_ref.x / n_norm;
    float n_y = mag_ref.y / n_norm;
    float n_z = mag_ref.z / n_norm;

    // Project magnetic North onto the horizontal plane (orthogonal to gravity) → X axis
    float dot_ng = n_x * z_x + n_y * z_y + n_z * z_z;
    float x_x = n_x - dot_ng * z_x;
    float x_y = n_y - dot_ng * z_y;
    float x_z = n_z - dot_ng * z_z;

    float x_norm = sqrtf(x_x * x_x + x_y * x_y + x_z * x_z);
    if (x_norm == 0.0f) {
        o.roll = o.pitch = o.yaw = 0.0f;
        return o;
    }
    x_x /= x_norm;
    x_y /= x_norm;
    x_z /= x_norm;

    // Y axis = Z × X
    float y_x = z_y * x_z - z_z * x_y;
    float y_y = z_z * x_x - z_x * x_z;
    float y_z = z_x * x_y - z_y * x_x;

    // Project current magnetic field into this reference frame
    float fwd   = m_x * x_x + m_y * x_y + m_z * x_z;  // forward
    float left  = m_x * y_x + m_y * y_y + m_z * y_z;  // left
    float down  = m_x * z_x + m_y * z_y + m_z * z_z;  // down

    // Orientation angles in launch reference frame
    o.yaw   = atan2f(left, fwd) * 180.0f / M_PI;
    o.pitch = atan2f(-down, sqrtf(fwd * fwd + left * left)) * 180.0f / M_PI;
    o.roll  = atan2f(left, down) * 180.0f / M_PI;  // Approximate roll

    // Normalize yaw to 0–360
    if (o.yaw < 0) o.yaw += 360.0f;

    return o;
}
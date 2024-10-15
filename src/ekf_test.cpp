// #include "FCMS.h"

// float phi_rad = 0;
// float theta_rad = 0;

// float P[4];
// float Q[2];
// float R[3];

// void init() {
//   P[0] = 0.1f; P[1] = 0.0f;
//   P[2] = 0.1f; P[3] = 0.0f;

//   Q[0] = 0.01f; Q[1] = 0.01f;
//   R[0] = 0.11f; R[1] = 0.11f; R[2] = 0.11f;
// }


// void predict(float p, float q, float r, float T) {
//     // Update state estimate (phi and theta angles)
//     float sp = sin(phi_rad);
//     float cp = cos(phi_rad);
//     float tt = tan(theta_rad);

//     // Update the angles using the gyroscope data (p, q, r)
//     phi_rad = phi_rad + T * (p + tt * (q * sp + r * cp));
//     theta_rad = theta_rad + T * (q * cp - r * sp);

//     // Recalculate trigonometric terms
//     sp = sin(phi_rad);
//     cp = cos(phi_rad);
//     float st = sin(theta_rad);
//     float ct = cos(theta_rad);

//     // Calculate the Jacobian matrix A based on the linearization of the system
//     float A[4] = {
//         tt * (q * cp - r * sp),   // A[0,0]
//         (r * cp + q * sp) * (tt * tt + 1.0f),  // A[0,1]
//         -(r * cp + q * sp),       // A[1,0]
//         0.0f                      // A[1,1]
//     };

//     // Initialize Ptmp for the predicted covariance matrix
//     float Ptmp[4] = {
//         T * (Q[0] + 2.0f * A[0] * P[0] + A[1] * P[1] + A[1] * P[2]),
//         T * (A[0] * P[1] + A[2] * P[0] + A[1] * P[3] + A[3] * P[2]),
//         T * (A[0] * P[1] + A[2] * P[0] + A[1] * P[3] + A[3] * P[2]),
//         T * (Q[1] + A[2] * P[1] + A[2] * P[1] + 2.0f * A[3] * P[3])
//     };

//     // Store the updated covariance back in P
//     P[0] = Ptmp[0];
//     P[1] = Ptmp[1];
//     P[2] = Ptmp[2];
//     P[3] = Ptmp[3];
// }

// void update(float ax, float ay, float az) {
//     // Step 1: Calculate the trigonometric values
//     float sp = sin(phi_rad);
//     float cp = cos(phi_rad);
//     float st = sin(theta_rad);
//     float ct = cos(theta_rad);
//     float g = 9.81f;

//     // Step 2: Output function h(x,u)
//     float h[3] = {
//         g * st,
//         -g * ct * sp,
//         -g * ct * cp
//     };

//     // Step 3: Jacobian of h(x,u)
//     float C[6] = {
//         0.0f, g * ct,
//         -g * cp * ct, g * sp * st,
//         g * sp * ct, g * cp * st
//     };

//     // Step 4: Kalman gain K = P * C' / (C * P * C' + R)
//     float G[9] = {
//         C[1] * (P[0] * C[1] + P[1] * C[2]) + C[3] * (P[2] * C[1] + P[3] * C[2]) + R[0],
//         C[1] * (P[0] * C[3] + P[1] * C[4]) + C[3] * (P[2] * C[3] + P[3] * C[4]) + R[1],
//         C[1] * (P[0] * C[5] + P[1] * C[5]) + C[3] * (P[2] * C[5] + P[3] * C[5]),
//         C[4] * (P[0] * C[1] + P[1] * C[2]) + C[4] * (P[2] * C[1] + P[3] * C[2]) + R[1],
//         C[4] * (P[0] * C[3] + P[1] * C[4]) + C[4] * (P[2] * C[3] + P[3] * C[4]) + R[2],
//         C[4] * (P[0] * C[5] + P[1] * C[5]) + C[4] * (P[2] * C[5] + P[3] * C[5]),
//         C[5] * (P[0] * C[1] + P[1] * C[2]) + C[5] * (P[2] * C[1] + P[3] * C[2]),
//         C[5] * (P[0] * C[3] + P[1] * C[4]) + C[5] * (P[2] * C[3] + P[3] * C[4]),
//         C[5] * (P[0] * C[5] + P[1] * C[5]) + C[5] * (P[2] * C[5] + P[3] * C[5]) + R[2]
//     };

//     // Step 5: Compute the determinant of G and its inverse
//     float Gdetinv = 1.0f / (G[0] * G[4] * G[8] - G[0] * G[5] * G[7] - G[1] * G[3] * G[8] + G[1] * G[5] * G[6]);

//     float Ginv[9] = {
//         Gdetinv * (G[4] * G[8] - G[5] * G[7]),
//         -Gdetinv * (G[3] * G[8] - G[5] * G[6]),
//         Gdetinv * (G[3] * G[7] - G[4] * G[6]),
//         -Gdetinv * (G[1] * G[8] - G[2] * G[7]),
//         Gdetinv * (G[0] * G[8] - G[2] * G[6]),
//         -Gdetinv * (G[0] * G[7] - G[1] * G[6]),
//         Gdetinv * (G[1] * G[5] - G[2] * G[4]),
//         -Gdetinv * (G[0] * G[5] - G[2] * G[3]),
//         Gdetinv * (G[0] * G[4] - G[1] * G[3])
//     };

//     // Step 6: Kalman gain matrix
//     float K[6] = {
//         Ginv[0] * (C[2] * P[0] + C[3] * P[1]) + Ginv[4] * (C[4] * P[0] + C[5] * P[1]),
//         Ginv[1] * (C[2] * P[0] + C[3] * P[1]) + Ginv[5] * (C[4] * P[0] + C[5] * P[1]),
//         Ginv[2] * (C[2] * P[0] + C[3] * P[1]) + Ginv[6] * (C[4] * P[0] + C[5] * P[1]),
//         Ginv[3] * (C[2] * P[2] + C[3] * P[3]) + Ginv[7] * (C[4] * P[2] + C[5] * P[3]),
//         Ginv[4] * (C[2] * P[2] + C[3] * P[3]) + Ginv[8] * (C[4] * P[2] + C[5] * P[3]),
//         Ginv[5] * (C[2] * P[2] + C[3] * P[3]) + Ginv[9] * (C[4] * P[2] + C[5] * P[3])
//     };

//     // Step 7: Update the covariance matrix P = (I - K * C) * P + Ptmp
//     float Ptmp[4] = {
//         -P[3] * (C[1] * K[1] + C[3] * K[1] + C[5] * K[2]),
//         -P[1] * (C[2] * K[0] + C[3] * K[1] + C[4] * K[2]),
//         -P[2] * (C[2] * K[0] + C[3] * K[1] + C[5] * K[2]),
//         -P[3] * (C[2] * K[0] + C[3] * K[1] + C[5] * K[2])
//     };

//     P[0] = P[0] + Ptmp[0];
//     P[1] = P[1] + Ptmp[1];
//     P[2] = P[2] + Ptmp[2];
//     P[3] = P[3] + Ptmp[3];

//     // Step 8: Update the state estimate x = x + K * (y - h)
//     phi_rad = phi_rad + K[0] * (ax - h[0]) + K[1] * (ay - h[1]) + K[2] * (az - h[2]);
//     theta_rad = theta_rad + K[3] * (ax - h[0]) + K[4] * (ay - h[1]) + K[5] * (az - h[2]);
// }
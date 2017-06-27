/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#pragma once

#include <armadillo>

class KalmanFilter
{
public:
  KalmanFilter(uint dim_in, uint dim_out, uint dim_state) : l(dim_in), m(dim_out), n(dim_state) {
    using arma::mat;
    using arma::vec;

    A = mat(n,n).eye();
    B = mat(n,l).zeros();
    C = mat(m,n).zeros();

    Q = mat(n,n).eye();
    R = mat(m,m).eye();
    P = mat(n,n).eye();

    K = mat(n,m).eye();
    I = arma::eye<mat>(n,n);

    u = vec(l).zeros();
    q_pred = vec(n).zeros();
    q_est = vec(n).zeros();
    y = vec(m).zeros();
  }

  void predictState() {
    q_pred = A * q_est + B * u;
    P = A * P * trans(A) + Q;
  }

  void correctState() {
    K = P * trans(C) * inv(C * P * trans(C) + R);
    q_est = q_pred + K * (y - C * q_pred);
    P = (I - K * C) * P;
  }

  void updateState() {
    predictState();
    correctState();
  }

public:
  // System matrices:
  arma::mat A;       // State
  arma::mat B;       // Input
  arma::mat C;       // Output

  // Covariance matrices:
  arma::mat Q;       // Process
  arma::mat R;       // Measurement
  arma::mat P;       // Estimate error

  // Kalman gain matrix:
  arma::mat K;

  // Identity matrix
  arma::mat I;

  // Signals:
  arma::vec u;       // Input
  arma::vec q_pred;  // Predicted state
  arma::vec q_est;   // Estimated state
  arma::vec y;       // Measurement

private:
  // Dimensions:
  uint l;             // Input
  uint m;             // Output
  uint n;             // State
};

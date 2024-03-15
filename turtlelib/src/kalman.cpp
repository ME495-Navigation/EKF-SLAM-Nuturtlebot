#include "turtlelib/kalman.hpp"
#include <cstdio>
#include <armadillo>

namespace turtlelib
{
    KalmanFilter::KalmanFilter()
    {
        // state vectors
        xk = arma::vec((2*N+3), arma::fill::zeros);
        xk_predicted = xk;
        // covariance matrices
        arma::mat rob_covar(3,3, arma::fill::zeros);
        // measurement covariance matrices
        arma::mat meas_covar(2*N,2*N, arma::fill::eye);
        // meas_covar is really high because we don't know the exact values
        meas_covar = 100000.0*meas_covar;
        // add the diagonal matrix to the covariance matrix
        arma::mat z1(3,2*N,arma::fill::zeros);
        arma::mat z2(2*N,3,arma::fill::zeros);
        covar = arma::join_cols(arma::join_rows(rob_covar, z1), arma::join_rows(z2, meas_covar));

        // predicted covar
        covar_predicted = covar;

        // twists
        twist_old = Twist2D{0.0, 0.0, 0.0};

        // detected obstacles default to false
        obstacle_detected = std::vector<bool>(N, false);
    }

    arma::vec KalmanFilter::get_state_estimate()
    {
        return xk;
    }

    void KalmanFilter::predict(Twist2D u)
    {
        // calculate del twist
        Twist2D u_del = Twist2D{normalize_angle(u.omega - twist_old.omega), u.x - twist_old.x, u.y - twist_old.y};
        // update the state
        xk_predicted(0) = normalize_angle(xk(0) + u_del.omega);
        xk_predicted(1) = xk_predicted(1) + u_del.x*cos(xk(0));
        xk_predicted(2) = xk_predicted(2) + u_del.x*sin(xk(0));
        // update twist_old
        twist_old = u;

        // A matrix
        arma::mat A = arma::mat((2*N+3),(2*N+3), arma::fill::eye);
        double theta = normalize_angle(xk(0));
        // fill in A matrix
        if(almost_equal(u_del.omega, 0.0, 1e-9))
        {
            A(1,0) = -u_del.x*sin(theta);
            A(2,0) = u_del.x*cos(theta);
        }
        else
        {
            A(1,0) = (-u_del.x/u_del.omega)*cos(theta) + (u_del.x/u_del.omega)*cos(normalize_angle(theta + u_del.omega));
            A(2,0) = (-u_del.x/u_del.omega)*sin(theta) + (u_del.x/u_del.omega)*sin(normalize_angle(theta + u_del.omega));
        }

        // calculate Q matrix
        arma::mat Q(3,3, arma::fill::eye);
        // multiply by noise
        Q = 0.011*Q;
        // calculate the Q_bar matrix
        arma::mat Q_bar = arma::join_cols(arma::join_rows(Q, arma::zeros(3,2*N)), arma::join_rows(arma::zeros(2*N,3), arma::zeros(2*N,2*N)));

        // update the covariance matrix
        covar_predicted = A*covar*A.t() + Q_bar;
    }

    void KalmanFilter::update(double o_x, double o_y, int j)
    {
        // calculate distance and angle to the obstacle
        double r_j = sqrt(pow(o_x, 2) + pow(o_y, 2));
        double phi_j = normalize_angle(atan2(o_y, o_x));

        // if the landmark hasn't been observed before, update its position in the state vector
        if (!obstacle_detected.at(j)) {
            obstacle_detected.at(j) = true;     // mark landmark as seen
            // innovation part: updating landmark position based on current observation
            xk_predicted(3 + 2 * j) = xk_predicted(1) + r_j *
            cos(normalize_angle(phi_j + xk_predicted(0)));
            xk_predicted(3 + 2 * j + 1) = xk_predicted(2) + r_j *
            sin(normalize_angle(phi_j + xk_predicted(0)));
        }

        // compute expected measurement (zbar) based on current state estimate
        Vector2D diff = {
            xk_predicted(3 + 2 * j) - xk_predicted(1),
            xk_predicted(3 + 2 * j + 1) - xk_predicted(2)
        };
        double dist = sqrt(pow(diff.x, 2) + pow(diff.y, 2));
        double phi = normalize_angle(atan2(diff.y, diff.x) - xk_predicted(0));
        arma::vec zbar = {dist, phi};   // expected measurement vector

        // construct the measurement model jacobian (H matrix)
        arma::mat H = Jacob(diff, dist, j);

        // compute the Kalman gain
        arma::mat R = arma::mat(2, 2, arma::fill::eye) * 0.1;   // measurement noise
        arma::mat K = covar_predicted * H.t() * arma::inv(H * covar_predicted * H.t() + R);   // Kalman gain calculation

        // compute innovation: the difference between actual and expected measurement
        arma::vec z_diff = arma::vec{r_j, phi_j} - zbar;
        z_diff(1) = normalize_angle(z_diff(1));

        // update state estimate with innovation
        xk_predicted += K * z_diff;
        xk_predicted(0) = normalize_angle(xk_predicted(0));

        // update covariance matrix
        arma::mat I = arma::eye<arma::mat>(2 * N + 3, 2 * N + 3);
        covar_predicted = (I - K * H) * covar_predicted;

        // align internal state and covariance with the updated predictions
        xk = xk_predicted;
        covar = covar_predicted;
    }

    arma::mat KalmanFilter::Jacob(const Vector2D & diff, double d, int j)
    {
        // calculate each component of H matrix
        arma::mat h1 = {
            {0.0, -diff.x / d, -diff.y / d},
            {-1.0, diff.y / pow(d, 2), -diff.x / pow(d, 2)}
        };
        arma::mat zeros_before(2, 2 * j, arma::fill::zeros);
        arma::mat h2 = {
            {diff.x / d, diff.y / d},
            {-diff.y / pow(d, 2), diff.x / pow(d, 2)}
        };
        arma::mat zeros_after(2, 2 * N - 2 * (j + 1), arma::fill::zeros);

        // construct and return the full H matrix
        return arma::join_rows(h1, zeros_before, h2, zeros_after); 
    }
}

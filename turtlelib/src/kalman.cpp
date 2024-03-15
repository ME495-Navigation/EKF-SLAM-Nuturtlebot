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

    arma::vec KalmanFilter::get_state_estimate() const
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

    void KalmanFilter::update(const Vector2D & state, int j)
    {
        // calculate distance and angle to the obstacle
        double d_obs = sqrt(pow(state.x, 2) + pow(state.y, 2));
        double phi_obs = normalize_angle(atan2(state.y, state.x));

        // update position of obstacle in state vector if it is detected and has not been detected before
        if(!obstacle_detected.at(j))
        {
            // update obstacle as detected
            obstacle_detected.at(j) = true;
            // update obs position based on distance and angle
            xk_predicted(3+2*j) = xk_predicted(1) + d_obs*cos(normalize_angle(xk_predicted(0) + phi_obs));
            xk_predicted(3+2*j+1) = xk_predicted(2) + d_obs*sin(normalize_angle(xk_predicted(0) + phi_obs));
        }

        // calculate expected measurement based on estimated state
        Vector2D diff = {xk_predicted(3+2*j) - xk_predicted(1), xk_predicted(3+2*j+1) - xk_predicted(2)};

        double d = sqrt(pow(diff.x, 2) + pow(diff.y, 2));
        double phi = normalize_angle(atan2(diff.y, diff.x) - xk_predicted(0));
        arma::vec z_bar = {d, phi};

        // calculate gain for Kalman filter
        arma::mat R = arma::mat(2,2, arma::fill::eye) * 0.1;
        arma::mat K = covar_predicted * H.t() * arma::inv(H * covar_predicted * H.t() + R);

        // calculate del between actual and expected measurement
        arma::vec z_del = arma::vec{d_obs, phi_obs} - z_bar;
        z_del(1) = normalize_angle(z_del(1));

        // update state estimate with z_del
        xk_predicted = xk_predicted + K*z_del;
        xk_predicted(0) = normalize_angle(xk_predicted(0));

        // update covariance matrix
        arma::mat I = arma::eye<arma::mat>(2*N+3, 2*N+3);
        covar_predicted = (I - K*H)*covar_predicted;

        // update state and covar
        xk = xk_predicted;
        covar = covar_predicted;
    }

    arma::mat KalmanFilter::Jacob(const Vector2D & diff, double d, int j)
    {
        // calculate each component of H matrix
        arma::mat h1 = {
            {0.0, -diff.x/d, -diff.y/d},
            {-1.0, diff.y/pow(d, 2), -diff.x/pow(d, 2)}
        };

        arma::mat h2 = {
            {diff.x/d, diff.y/d},
            {-diff.y/pow(d, 2), diff.x/pow(d, 2)}
        };

        // zeros
        arma::mat z1(2, 2*j, arma::fill::zeros);
        arma::mat z2(2, 2*N-2*(j+1), arma::fill::zeros);

        // join components
        return arma::join_rows(h1,z1,h2,z2); 
    }
}

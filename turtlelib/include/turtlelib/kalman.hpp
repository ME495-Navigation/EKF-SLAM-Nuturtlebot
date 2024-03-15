#ifndef KALMAN_HPP_INCLUDE_GUARD
#define KALMAN_HPP_INCLUDE_GUARD
/// \file
/// \brief Extended Kalman filter for 2D pose estimation.

// include necessary headers/libraries
#include "turtlelib/diff_drive.hpp"
#include <armadillo>

namespace turtlelib
{
    /// \brief number of obstacles
    constexpr int N = 25;

    /// \brief Extended Kalman filter for 2D pose estimation.
    class KalmanFilter
    {
    private:
        /// \brief xk_predicted - the predicted state
        arma::vec xk_predicted;
        arma::vec xk;
        arma::mat covar_predicted;
        arma::mat covar;
        arma::mat H;
        arma::mat K;
        arma::mat Q;
        arma::mat R;
        arma::mat Jacob(const Vector2D & diff, double d, int j);

        Twist2D twist_old;
        std::vector<bool> obstacle_detected;


    public:
        /// \brief Create EKF constructor
        KalmanFilter();

        /// \brief xk_predicted - the predicted state
        /// \return the predicted state
        arma::vec get_state_estimate() const;

        /// \brief Pk_predicted - the predicted covariance
        void predict(Twist2D u);

        /// \brief update the state estimate
        void update(const Vector2D & state, int j);
        
    };
}




#endif
#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>
#include <tuple>
#include <iostream>

namespace lqr_controller {
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;
typedef std::vector<VectorXd, Eigen::aligned_allocator<VectorXd>> VecOfVectorXd;
typedef std::vector<MatrixXd, Eigen::aligned_allocator<MatrixXd>> VecOfMatrixXd;
class Problem {
public:
    Problem(int nbSteps, VectorXd X_r) : nbSteps_(nbSteps), X_r_(X_r)  {
        init();
    }
    ~Problem() = default;

    void init() {
        dt_ = 0.1;
        n_ = 3;
        m_ = 2;
        nbIter_ = 100;
        Q_ = 1e2 * MatrixXd::Identity(n_ * nbSteps_, n_ * nbSteps_);
        Q_.bottomRightCorner(n_, n_) = 1e3 * MatrixXd::Identity(n_, n_);
        R_ = MatrixXd::Identity(m_ * (nbSteps_ - 1), m_ * (nbSteps_ - 1));
    }
    inline VectorXd flatten(const MatrixXd& M) {
        MatrixXd M_T = M.transpose();
        return Eigen::Map<VectorXd>(M_T.data(), M_T.size());
    }
    inline MatrixXd reshape(const VectorXd& v, unsigned int cols, unsigned int rows) {
        return Eigen::Map<const MatrixXd>(v.data(), rows, cols).transpose();
    }
    inline VectorXd f(const VectorXd& x, const VectorXd& u) {
        VectorXd xnext(n_);
        xnext(0) = x(0) + dt_ * u(0) * cos(x(2));
        xnext(1) = x(1) + dt_ * u(0) * sin(x(2));
        xnext(2) = x(2) + dt_ * u(1);
        return xnext;
    }
    inline MatrixXd get_jacobian_A(const VectorXd& x, const VectorXd& u) {
        MatrixXd A = MatrixXd::Identity(n_, n_);
        A(0, 2) = - dt_ * u(0) * sin(x(2));
        A(1, 2) = dt_ * u(0) * cos(x(2));
        return A;
    }
    inline MatrixXd get_jacobian_B(const VectorXd& x, const VectorXd& /*u*/) {
        MatrixXd B = MatrixXd::Zero(n_, m_);
        B(0, 0) = dt_ * cos(x(2));
        B(1, 0) = dt_ * sin(x(2));
        B(2, 1) = dt_;
        return B;
    }

    inline MatrixXd rollout(const VectorXd& x_init, const MatrixXd& U) {
        unsigned int nbSteps = U.rows()+1;
        MatrixXd X = MatrixXd::Zero(nbSteps, n_);
        X.row(0) = x_init;
        for(unsigned int i = 0; i < nbSteps-1; i++) {
            X.row(i+1) = f(X.row(i), U.row(i));
        }
        return X;
    }

    inline MatrixXd get_Su(const MatrixXd& X, const MatrixXd& U) {
        unsigned int nbSteps = X.rows();
        MatrixXd Su = MatrixXd::Zero(n_*nbSteps, m_*(nbSteps-1));
        for(unsigned int j = 0; j < nbSteps-1; j++) {
            Su.block((j+1)*n_, j*m_, n_, m_) = get_jacobian_B(X.row(j), U.row(j));
            for(unsigned int i = 0; i < nbSteps-2-j; i++) {
                Su.block((j+m_+i)*n_, j*m_, n_, m_) = get_jacobian_A(X.row(i+j+1), U.row(i+j+1)) * Su.block((j+1+i)*n_, j*m_, n_, m_);
            }
        }
        return Su;
    }
    // Cost function
    // ===============================
    inline double cost(const MatrixXd& X, const MatrixXd& U) {
        return (flatten(X) - X_r_).dot(Q_ * (flatten(X) - X_r_)) + flatten(U).dot(R_ * flatten(U));
    }
    std::tuple<std::vector<VectorXd>, MatrixXd> lqrSolve(const VectorXd& x_init);
private:
    int n_, m_;
    double dt_;
    MatrixXd Q_, R_;
    // double Q_t_, Q_T_, R_t_;
    unsigned int nbIter_, nbSteps_;
    VectorXd x_r_;
    VectorXd X_r_;
};

std::tuple<std::vector<VectorXd>, MatrixXd> Problem::lqrSolve(const VectorXd& x_init) {
    MatrixXd U = MatrixXd::Zero(nbSteps_ - 1, m_);
    for(unsigned int k = 0; k < nbIter_; k++) {
        MatrixXd X = rollout(x_init, U);            
        double current_cost = cost(X, U);
        MatrixXd Su = get_Su(X, U);
        VectorXd delta_u = (Su.transpose() * Q_ * Su + R_).llt().solve(Su.transpose() * Q_ * (X_r_ - flatten(X)) - R_ * flatten(U));
        // Line search
        double alpha = 1.0;
        double best_cost = current_cost;
        MatrixXd U_best = U;
        for(unsigned int i = 0; i < 10; i++) {
            VectorXd u_tmp = flatten(U) + alpha * delta_u;
            MatrixXd U_tmp = reshape(u_tmp, nbSteps_ - 1, m_);
            X = rollout(x_init, U_tmp);
            double cost_tmp = cost(X, U_tmp);
            if(cost_tmp < best_cost) {
                best_cost = cost_tmp;
                U_best = U_tmp;
            }
            alpha = alpha / 2.;
        }
        
        if((flatten(U) - flatten(U_best)).squaredNorm() < 1e-3) {
            U = U_best;
            break;
        }
        U = U_best;
        if(k == nbIter_ - 1) {
            std::cout << "WARNING: LQR did not converge" << std::endl;
        }
    }
    MatrixXd X = rollout(x_init, U);
    std::vector<VectorXd> X_vec(nbSteps_);
    for(unsigned int i = 0; i < nbSteps_; i++) {
        X_vec[i] = X.row(i);
    }
    return std::make_tuple(X_vec, U);
}
}

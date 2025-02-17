#include <iostream>
#include "lqr_controller/LQR.hpp"
 
using namespace std;
 
void LQR::initial(double L_, double T_, vehicleState car, waypoint waypoint, U U_r, double *Q_, double *R_) {
 
	L = L_;
	T = T_;
	x_car = car.x; y_car = car.y; yaw_car = car.yaw;v_car = car.v;
	x_d = waypoint.x; y_d = waypoint.y; yaw_d = waypoint.yaw;
	v_d = U_r.v;kesi_d = U_r.kesi;
 
	for (int i = 0; i < 5; i++) {
		Q5[i] = Q_[i];
	}
	for (int j = 0; j < 2; j++) {
		R2[j] = R_[j];
	}
}
 
void LQR::param_struct() {
	
	Q << 	Q5[0], 0.0,   0.0,  0.0,  0.0,
			0.0,   Q5[1], 0.0,  0.0,  0.0,
			0.0,   0.0,   Q5[2],0.0,  0.0,
			0.0,   0.0,   0.0,  Q5[3],0.0,
			0.0,   0.0,   0.0,  0.0,  Q5[4];
	// cout << "Q矩阵为：\n" << Q << endl;
	R << R2[0], 0.0,
		 0.0,   R2[1];
	// cout << "R矩阵为：\n" << R << endl;
	A_d <<  1.0, T,   0,   0,   0.0,
			0.0, 0.0, v_car, 0.0, 0.0,
			0.0, 0.0, 1.0, T,   0.0,
			0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 1.0;
	
	Eigen::JacobiSVD<Matrix5x5> svd(A_d);
	// cout << "A_d rank:" << svd.rank() << endl;
	// cout << "A_d矩阵为:\n" << A_d << endl;
	B_d << 	0,  	0,
			0,		0,
			0,		0,
			v_car/L,	0.0,
			0.0,	T;
	// cout << "B_d矩阵为:\n" << B_d << endl;
	
	
}
 
Matrix2x5 LQR::cal_Riccati() {
	int N = 1000;//迭代终止次数
	double err = 100;//误差值
	double err_tolerance = 0.01;//误差收敛阈值
	Matrix5x5 Qf = Q;
	Matrix5x5 P = Qf;//迭代初始值
	//cout << "P初始矩阵为\n" << P << endl;
	Matrix5x5 Pn;//计算的最新P矩阵
	
	for (int iter_num = 0; iter_num < N; iter_num++) {
		Pn = Q + A_d.transpose() * P * A_d - A_d.transpose() * P * B_d * (R + B_d.transpose() * P * B_d).inverse() * B_d.transpose() * P * A_d;//迭代公式
		// cout << "收敛误差为" << (Pn - P).array().abs().maxCoeff() << endl;
		// cout << Pn << endl;
		err = (Pn - P).array().abs().maxCoeff();//
		err = (Pn - P).lpNorm<Eigen::Infinity>();
		// cout << "itercout:" << iter_num << "误差为" << err << endl;
		if(err < err_tolerance)//
		{
			P = Pn;
			cout << "迭代次数" << iter_num << endl;
			break;
		}
		P = Pn;
	}
	
	//cout << "P矩阵为\n" << P << endl;
	//P = Q;
	Matrix2x5 K = (R + B_d.transpose() * P * B_d).inverse() * B_d.transpose() * P * A_d;//反馈率K
	return K;
}
 
U LQR::cal_vel() {
	U output;
	// cout << "param struct" << endl;
	param_struct();
	// cout << "calculate riccati" << endl;
	Matrix2x5 K = cal_Riccati();

	static double last_e = 0,last_e_th = 0;

	Matrix4x4 T_map_robot,T_map_ref_p;

	T_map_robot <<  cos(yaw_car), -sin(yaw_car), 0 , x_car, 
					sin(yaw_car),  cos(yaw_car), 0 , y_car, 
					0, 					0	, 		  1,  0,
					0, 					0	, 		  0,  1	;

	cout << "T_map_robot矩阵为:\n" << T_map_robot << endl;

	T_map_ref_p << cos(yaw_d),  -sin(yaw_d), 0 , x_d, 
					sin(yaw_d),  cos(yaw_d), 0 , y_d, 
					0, 					0	, 		  1,  0,
					0, 					0	, 		  0,  1	;

	cout << "T_map_ref_p矩阵为:\n" << T_map_ref_p << endl;

	Matrix4x4 T_p_robot = T_map_ref_p.inverse() * T_map_robot;

	cout << "T_p_robot矩阵为:\n" << T_p_robot << endl;

	double e = T_p_robot(1,3);//hypot(x_car - x_d, y_car - y_d);
	// double e = hypot(x_car - x_d, y_car - y_d);
	// double dxl = x_d - x_car;
	// double dyl = y_d - y_car;
	// double angle_diff = YAW_P2P(yaw_d - atan2(dyl,dxl));

	// if(angle_diff<0)e = e*-1;
	
	double e_dot = (e - last_e)/T;
	double e_th = YAW_P2P(yaw_car - yaw_d);
	double e_th_dot = (e_th - last_e_th)/T;
	last_e = e;
	last_e_th = e_th;

	X_e << e , 
		   e_dot,
		   e_th,
		   e_th_dot,
		   v_car-v_d;
	
	cout << "X_e矩阵为:\n" << X_e << endl;


	// cout << "compute U" << endl;
	Matrix2x1 U = -K * X_e;
	// cout << "反馈增益K为：\n" << K << endl;
	// cout << "控制输入U为：\n" << U << endl;
	
	output.v = U[1]* T + v_car;
	output.a = U[1];
	output.kesi = U[0] + kesi_d;
	return output;
}
 
void LQR::test() //控制器效果测试
{
	/*param_struct();
	while (temp < 1000) {
		Matrix2x3 K = cal_Riccati();
		Matrix2x1 U = K * X_e;
		//cout <<"state variable is:\n" <<X_e << endl;
		//cout <<"control input is:\n"<< U << endl;
		Matrix3x1 X_e_ = A_d * X_e + B_d * U;
		X_e = X_e_;
		temp++;
	}*/
	// Matrix3x3 C,D,F;
	// C << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
	// F << 1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 7.0;
	// D = (C - F);
	// double BBBB = D.lpNorm<Eigen::Infinity>();
	// cout << BBBB << endl;
}
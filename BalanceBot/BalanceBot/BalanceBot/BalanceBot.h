/*
 * BalanceBot.h
 *
 * Created: 4/15/2022 2:35:47 PM
 *  Author: Matthew
 */ 


#ifndef BALANCEBOT_H_
#define BALANCEBOT_H_

#include "MotorPWM.h"
#include "matrix.h"


struct Segway {
	float** K;
	float** L;
	float** xd;
	float** uLast;
	float** x_est;
	float** y_est;
	float** Ad;
	float** Bd;
	float** measError;
	float** y;
	float** stateError;
	float** u;
	float** Bu;
	float** Lm;
	} BalanceBot;
	
void BalanceBotInit(){
	BalanceBot.K = allocate_matrix(1,3);
	BalanceBot.K[0][0] = -10.0389432396441;
	BalanceBot.K[0][1] = -78.5368489286044;
	BalanceBot.K[0][2] = -5.22263746115749;
	BalanceBot.Bu = allocate_matrix(3,1);
	BalanceBot.Bu[0][0] = 0;
	BalanceBot.Bu[0][1] = 0;
	BalanceBot.Bu[0][2] = 0;
	BalanceBot.Lm = allocate_matrix(3,1);
	BalanceBot.Lm[0][0] = 0;
	BalanceBot.Lm[0][1] = 0;
	BalanceBot.Lm[0][2] = 0;
	BalanceBot.L = allocate_matrix(3,1);
	BalanceBot.L[0][0] = 0.0332983580509553;
	BalanceBot.L[1][0] = 0.0477709335719864;
	BalanceBot.L[2][0] = 0.862378691970006;
	BalanceBot.xd = allocate_matrix(3,1);
	BalanceBot.xd[0][0] = 0;
	BalanceBot.xd[1][0] = M_PI;
	BalanceBot.xd[2][0] = 0;
	BalanceBot.uLast = allocate_matrix(1,1);
	BalanceBot.uLast[0][0] = 0;
	BalanceBot.measError = allocate_matrix(1,1);
	BalanceBot.measError[0][0] = 0;
	BalanceBot.x_est = allocate_matrix(3,1);
	BalanceBot.x_est[0][0] = 0;
	BalanceBot.x_est[1][0] = M_PI;
	BalanceBot.x_est[2][0] = 0;
	BalanceBot.y_est = allocate_matrix(1,1);
	BalanceBot.y_est[0][0] = M_PI;
	BalanceBot.y = allocate_matrix(1,1);
	BalanceBot.y[0][0] = 0;
	BalanceBot.Ad = allocate_matrix(3,3);
	BalanceBot.Ad[0][0] = 0.991982012478150;
	BalanceBot.Ad[0][1] = 0.00509492179608207;
	BalanceBot.Ad[0][2] = 0.00000255083713775875;
	BalanceBot.Ad[1][0] = 0.0000223804723377318;
	BalanceBot.Ad[1][1] = 1.00009790913983;
	BalanceBot.Ad[1][2] = 0.00100003263298865;
	BalanceBot.Ad[2][0] = 0.0447016999369507;
	BalanceBot.Ad[2][1] = 0.195859583825295;
	BalanceBot.Ad[2][2] = 1.00009790913983;
	BalanceBot.Bd = allocate_matrix(3,1);
	BalanceBot.Bd[0][0] = 0.000909862410824434;
	BalanceBot.Bd[1][0] = -0.00000253968348804726;
	BalanceBot.Bd[2][0] = -0.00507264402217493;
	BalanceBot.stateError = allocate_matrix(3,1);
	BalanceBot.u = allocate_matrix(1,1);
}


#endif /* BALANCEBOT_H_ */
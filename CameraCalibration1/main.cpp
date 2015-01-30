/**
 * 非線形最小二乗法ライブラリ(lmdif)をテストするプログラム。
 * 与えられた９個の観測データを、ax=bの式にフィットさせる。
 * （線形の式を使っているけど、まぁテストなので。。。）
 *
 * @author	Gen Nishida
 * @version	1.0
 * @date	1/29/2015
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <cminpack.h>
#define real __cminpack_real__

// 観測データを定義する構造体
typedef struct {
	int m;
	real *y;
} fcndata_t;

int fcn(void *p, int m, int n, const real *x, real *fvec, int iflag);

int main() {
	// パラメータの数
	const int n = 2;

	// 観測データの数
	const int m = 9; 

	// パラメータ（n個）
	real x[n];

	// パラメータの初期推定値
	x[0] = 1.;
	x[1] = 1.;
	//x[2] = 1.;

	// 観測データ（m個）
	//real y[m] = {1.4e-1, 1.8e-1, 2.2e-1, 2.5e-1, 2.9e-1, 3.2e-1, 3.5e-1, 3.9e-1, 3.7e-1, 5.8e-1, 7.3e-1, 9.6e-1, 1.34, 2.1, 4.39};
	real y[m] = {10, 30, 20, 70, 30, 10, 80, 40, 60};

	// 真値と観測データとの誤差が格納される配列
	real fvec[m];

	// 結果のヤコビ行列
	real fjac[m*n];

	// lmdif内部使用パラメータ
	int ipvt[n];

	real diag[n], qtf[n], wa1[n], wa2[n], wa3[n], wa4[m];

	// 観測データを格納する構造体オブジェクト
	fcndata_t data;
	data.m = m;
	data.y = y;

	// 観測データの数と同じ値にすることを推奨する
	int ldfjac = m;

	// 各種パラメータ（推奨値のまま）
	real ftol = sqrt(__cminpack_func__(dpmpar)(1));
	real xtol = sqrt(__cminpack_func__(dpmpar)(1));
	real gtol = 0.;

	// 何回繰り返すか？
	int maxfev = 800;

	// 収束チェック用の微小値
	real epsfcn = 1e-08;
	int mode = 1;

	// 1が推奨されている？
	real factor = 1;//1.e2;

	// 実際に繰り返した回数
	int nfev;

	int nprint = 0;
	int info = __cminpack_func__(lmdif)(fcn, &data, m, n, x, fvec, ftol, xtol, gtol, maxfev, epsfcn,
									diag, mode, factor, nprint, &nfev, fjac, ldfjac, ipvt, qtf, wa1, wa2, wa3, wa4);
	real fnorm = __cminpack_func__(enorm)(m, fvec);

	printf(" final l2 norm of the residuals%15.7g\n\n", (double)fnorm);
	printf(" number of function evaluations%10i\n\n", nfev);
	printf(" exit parameter %10i\n\n", info);
	printf(" final approximate solution\n");
	for (int i = 0; i < n; ++i) {
		printf("%lf\t", (double)x[i]);
	}
	printf("\n");

	return 0;
}

/**
 * 自分の関数を記述し、真値と観測データとの差を計算する。
 *
 * @param p		観測データが入った構造体オブジェクト
 * @param m		観測データの数
 * @param n		パラメータの数
 * @param x		パラメータ配列
 * @param fvec	真値と観測データとの差を格納する配列
 * @param iflag	lmdifから返されるフラグ (0なら終了?)
 * @return		0を返却する
 */
int fcn(void *p, int m, int n, const real *x, real *fvec, int iflag) {
	const real *y = ((fcndata_t*)p)->y;
	//assert(m == 15 && n == 3);

	if (iflag == 0) {
		/* insert print statements here when nprint is positive. */
		/* if the nprint parameter to lmdif is positive, the function is
		called every nprint iterations with iflag=0, so that the
		function may perform special operations, such as printing
		residuals. */
		return 0;
	}

	// 真値と観測データの差を計算する
	/*for (int i = 0; i < 15; ++i) {
		real tmp1 = i + 1;
		real tmp2 = 15 - i;
		real tmp3 = (i > 7) ? tmp2 : tmp1;
		fvec[i] = y[i] - (x[0] + tmp1/(x[1]*tmp2 + x[2]*tmp3));
	}*/
	// a x + bのモデルにフィットさせてみる
	for (int i = 0; i < m; ++i) {
		real tmp1 = i * 10 + 10;
		fvec[i] = y[i] - (x[0] * tmp1 + x[1]);
	}

	return 0;
}
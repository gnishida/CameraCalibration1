/* driver for lmdif example. */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <cminpack.h>
#define real __cminpack_real__
#define TEST_COVAR

/* the following struct defines the data points */
typedef struct {
	int m;
	real *y;
} fcndata_t;

int fcn(void *p, int m, int n, const real *x, real *fvec, int iflag);

int main() {
	// 実際に繰り返した回数
	int nfev;

	int ipvt[3];

	// パラメータ（3個）
	real x[3];

	// 真値と観測データとの誤差が格納される配列
	real fvec[15];

	real diag[3], qtf[3], wa1[3], wa2[3], wa3[3], wa4[15];
	
	// 結果のヤコビ行列
	real fjac[15*3];

	// 観測データの数
	const int m = 15; 

	// パラメータの数
	const int n = 3;

	// 観測データ
	real y[15] = {1.4e-1, 1.8e-1, 2.2e-1, 2.5e-1, 2.9e-1, 3.2e-1, 3.5e-1, 3.9e-1, 3.7e-1, 5.8e-1, 7.3e-1, 9.6e-1, 1.34, 2.1, 4.39};
	
	// 観測データを格納する構造体オブジェクト
	fcndata_t data;
	data.m = m;
	data.y = y;

	// パラメータの初期推定値
	x[0] = 1.;
	x[1] = 1.;
	x[2] = 1.;

	// 観測データの数と同じ値にすることを推奨する
	int ldfjac = 15;

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
	/* subroutine fcn for lmdif example. */
	real tmp1, tmp2, tmp3;
	const real *y = ((fcndata_t*)p)->y;
	assert(m == 15 && n == 3);

	if (iflag == 0) {
		/* insert print statements here when nprint is positive. */
		/* if the nprint parameter to lmdif is positive, the function is
		called every nprint iterations with iflag=0, so that the
		function may perform special operations, such as printing
		residuals. */
		return 0;
	}

	/* compute residuals */
	for (int i = 0; i < 15; ++i) {
		tmp1 = i + 1;
		tmp2 = 15 - i;
		tmp3 = (i > 7) ? tmp2 : tmp1;
		fvec[i] = y[i] - (x[0] + tmp1/(x[1]*tmp2 + x[2]*tmp3));
	}

	return 0;
}
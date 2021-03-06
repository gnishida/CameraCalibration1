C     DRIVER FOR LMSTR EXAMPLE.
C     DOUBLE PRECISION VERSION
C
C     **********
      INTEGER J,M,N,LDFJAC,MAXFEV,MODE,NPRINT,INFO,NFEV,NJEV,NWRITE
      INTEGER IPVT(3)
      DOUBLE PRECISION FTOL,XTOL,GTOL,FACTOR,FNORM
      DOUBLE PRECISION X(3),FVEC(15),FJAC(3,3),DIAG(3),QTF(3),
     *                 WA1(3),WA2(3),WA3(3),WA4(15)
      DOUBLE PRECISION ENORM,DPMPAR
      EXTERNAL FCN
C
C     LOGICAL OUTPUT UNIT IS ASSUMED TO BE NUMBER 6.
C
      DATA NWRITE /6/
C
      M = 15
      N = 3
C
C     THE FOLLOWING STARTING VALUES PROVIDE A ROUGH FIT.
C
      X(1) = 1.D0
      X(2) = 1.D0
      X(3) = 1.D0
C
      LDFJAC = 3
C
C     SET FTOL AND XTOL TO THE SQUARE ROOT OF THE MACHINE PRECISION
C     AND GTOL TO ZERO. UNLESS HIGH PRECISION SOLUTIONS ARE
C     REQUIRED, THESE ARE THE RECOMMENDED SETTINGS.
C
      FTOL = DSQRT(DPMPAR(1))
      XTOL = DSQRT(DPMPAR(1))
      GTOL = 0.D0
C
      MAXFEV = 400
      MODE = 1
      FACTOR = 1.D2
      NPRINT = 0
C
      CALL LMSTR(FCN,M,N,X,FVEC,FJAC,LDFJAC,FTOL,XTOL,GTOL,
     *           MAXFEV,DIAG,MODE,FACTOR,NPRINT,INFO,NFEV,NJEV,
     *           IPVT,QTF,WA1,WA2,WA3,WA4)
      FNORM = ENORM(M,FVEC)
      WRITE (NWRITE,1000) FNORM,NFEV,NJEV,INFO,(X(J),J=1,N)
      STOP
 1000 FORMAT (5X,31H FINAL L2 NORM OF THE RESIDUALS,D15.7 //

     *        5X,31H NUMBER OF FUNCTION EVALUATIONS,I10 //
     *        5X,31H NUMBER OF JACOBIAN EVALUATIONS,I10 //
     *        5X,15H EXIT PARAMETER,16X,I10 //
     *        5X,27H FINAL APPROXIMATE SOLUTION // 5X,3D15.7)
C
C     LAST CARD OF DRIVER FOR LMSTR EXAMPLE.
C
      END
      SUBROUTINE FCN(M,N,X,FVEC,FJROW,IFLAG)
      INTEGER M,N,IFLAG
      DOUBLE PRECISION X(N),FVEC(M),FJROW(N)
C
C     SUBROUTINE FCN FOR LMSTR EXAMPLE.
C
      INTEGER I
      DOUBLE PRECISION TMP1,TMP2,TMP3,TMP4
      DOUBLE PRECISION Y(15)
      DATA Y(1),Y(2),Y(3),Y(4),Y(5),Y(6),Y(7),Y(8),
     *     Y(9),Y(10),Y(11),Y(12),Y(13),Y(14),Y(15)
     *     /1.4D-1,1.8D-1,2.2D-1,2.5D-1,2.9D-1,3.2D-1,3.5D-1,3.9D-1,
     *      3.7D-1,5.8D-1,7.3D-1,9.6D-1,1.34D0,2.1D0,4.39D0/
C
      IF (IFLAG .NE. 0) GO TO 5
C
C     INSERT PRINT STATEMENTS HERE WHEN NPRINT IS POSITIVE.
C
      RETURN
    5 CONTINUE
      IF (IFLAG .GE. 2) GO TO 20
      DO 10 I = 1, 15
         TMP1 = I
         TMP2 = 16 - I
         TMP3 = TMP1
         IF (I .GT. 8) TMP3 = TMP2
         FVEC(I) = Y(I) - (X(1) + TMP1/(X(2)*TMP2 + X(3)*TMP3))
   10    CONTINUE
      GO TO 40
   20 CONTINUE
      I = IFLAG - 1
         TMP1 = I
         TMP2 = 16 - I
         TMP3 = TMP1
         IF (I .GT. 8) TMP3 = TMP2
         TMP4 = (X(2)*TMP2 + X(3)*TMP3)**2
         FJROW(1) = -1.D0
         FJROW(2) = TMP1*TMP2/TMP4
         FJROW(3) = TMP1*TMP3/TMP4
   30    CONTINUE
   40 CONTINUE
      RETURN
C
C     LAST CARD OF SUBROUTINE FCN.
C
      END

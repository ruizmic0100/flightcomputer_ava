void KalmanRollPitch_Init(KalmanRollPitch *kal, float Pinit, float *Q, float *R) {
	kal->phi   = 0.0f;
	kal->theta = 0.0f;
	kal->P[0] = Pinit; kal->P[1] = 0.0f;
	kal->P[2] = 0.0f;  kal->P[3] = Pinit;
	kal->Q[0] = Q[0];  kal->Q[1] = Q[1];
	kal->R[0] = R[0];  kal->R[1] = R[1]; kal->R[2] = R[2];
	kal->gyr[0] = 0.0f;
	kal->gyr[1] = 0.0f;
	kal->gyr[2] = 0.0f;
}

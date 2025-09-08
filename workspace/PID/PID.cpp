//速度調整PID制御
float PID(float v,float a_meas,float cycle_ms){
  float target_v=100.0f; //目標速度[cm/s]
  float KP=0.8f;
  float KI=0.6f;
  float KD=0.5f;
  const float OUT_MIN=0.0f, OUT_MAX=100.0f; //出力範囲
  const float I_MAX=400.0f; //積分上限
  static float I=0.0f; //積分状態

  float dt=cycle_ms*0.001f; //Δt、ms→s
  if(dt<0.0001f){
    dt=0.0001f;
  }

  //P制御:e=target_v−v
  float e = target_v-v;

  //I制御:I←I+e·Δt
  I = I + e * dt;
  if(I > I_MAX){
    I = I_MAX;
  }else if(I < -I_MAX){
    I = -I_MAX;
  }

  //D制御:測定した加速度を使用

  float out_power=KP*e+KI*I-KD*a_meas; 
  //出力= Kp・e +　Ki・I − Kd・a

  //出力制限
  if(out_power>OUT_MAX){
    out_power=OUT_MAX;
  }else if(out_power<OUT_MIN){
    out_power=OUT_MIN;
  }

  return out_power;
}
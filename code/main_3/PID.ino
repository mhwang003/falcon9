float roll_PID(float ref_Roll, float aroll){
  e_roll = ref_Roll - aroll;
  eInt_roll += e_roll/1;
  eDiff_roll = e_roll - ePrev_roll;
  ePrev_roll = e_roll;
  int uroll = e_roll*KP_NUM/KP_DEN;// + eInt*KI_NUM/KP_DEN;
  
  return uroll;
}


float pitch_PID(float ref_Pitch, float apitch){
  e_pitch = ref_Pitch - apitch;
  eInt_pitch += e_pitch/1;
  eDiff_pitch = e_pitch - ePrev_pitch;
  ePrev_pitch = e_pitch;
  int upitch = e_pitch*KP_NUM/KP_DEN;// + eInt*KI_NUM/KP_DEN;
  
  return upitch;
}

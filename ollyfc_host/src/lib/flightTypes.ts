export type FlightLogData = {
  timestamp: number;
  accel_x: number;
  accel_y: number;
  accel_z: number;
  pitch: number;
  yaw: number;
  roll: number;
  throttle: number;
  aileron: number;
  elevator: number;
  rudder: number;
  ctl_throttle: number;
  ctl_aileron: number
  ctl_elevator: number;
  ctl_rudder: number;
  arm: number;
  enable: number;
  record: number;
};

export type LogInfoPage = {
  blockStartPtr: number;
  blockEndPtr: number;
  blockSize: number;
  nBlocks: number;
  currentPage: number;
};
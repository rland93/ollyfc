export type SensorInput = {
  accel_x: number;
  accel_y: number;
  accel_z: number;
  pitch: number;
  yaw: number;
  roll: number;
};

export type SBusInput = {
  throttle: number;
  aileron: number;
  elevator: number;
  rudder: number;
  arm: number;
  enable: number;
  record: number;
};

export type ControlPolicy = {
  elevator: number;
  aileron: number;
  rudder: number;
  throttle: number;
};

export type FlightLogData = {
  timestamp: number;
  sbus_input: SBusInput;
  sensor_input: SensorInput;
  control_policy: ControlPolicy;
};

export type LogInfoPage = {
  blockStartPtr: number;
  blockEndPtr: number;
  blockSize: number;
  nBlocks: number;
  currentPage: number;
};

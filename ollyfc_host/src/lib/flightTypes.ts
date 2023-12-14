export type SensorInput = {
  accelX: number;
  accelY: number;
  accelZ: number;
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
  sbusInput: SBusInput;
  sensorInput: SensorInput;
  controlPolicy: ControlPolicy;
};
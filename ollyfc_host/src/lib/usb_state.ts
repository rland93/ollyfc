import { invoke } from "@tauri-apps/api/primitives";
import { writable } from "svelte/store";
import type { FlightLogData } from "./flightTypes";


/***** Types *****/

export type UsbDataDisplay = {
  cmd: string;
  data: string;
};

export type LogDumpProgress = {
  current: number;
  total: number;
};



/***** Stores *****/

export let usb = writable<boolean>(false);
export let logdata = writable<FlightLogData[]>([]);

/***** Functions *****/

export async function searchUSB() {
  let res: boolean = await invoke("search_for_usb");
  usb.set(res);
}

export async function disconnect() {
  let res: boolean = await invoke("disconnect_usb");
  usb.set(res);
}


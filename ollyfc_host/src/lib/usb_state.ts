import { invoke } from "@tauri-apps/api/primitives";
import { writable } from "svelte/store";


/***** Types *****/

export type UsbDataDisplay = {
  cmd: string;
  data: string;
};


/***** Stores *****/

export let usb = writable<boolean>(false);


/***** Functions *****/

export async function searchUSB() {
  let res: boolean = await invoke("search_for_usb");
  usb.set(res);
}

export async function disconnect() {
  let res: boolean = await invoke("disconnect_usb");
  usb.set(res);
}


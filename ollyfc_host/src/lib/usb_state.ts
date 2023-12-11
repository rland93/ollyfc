import { invoke } from "@tauri-apps/api/primitives";
import { writable } from "svelte/store";

export let usb = writable<boolean>(false);

export async function searchUSB() {
  let res: boolean = await invoke("search_for_usb");
  usb.set(res);
}
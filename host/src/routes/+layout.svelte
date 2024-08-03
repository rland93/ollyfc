<script lang="ts">
    import "../global.scss";
    import { onMount, afterUpdate } from "svelte";
    import { listen } from "@tauri-apps/api/event";
    import { connectionStore } from "$lib/connectionStore";
    import { settingsStore } from "$lib/settingsStore";

    import { goto } from "$app/navigation";
    import { page } from "$app/stores";

    interface NavItem {
        href: string;
        label: string;
        requiresConnection: boolean;
    }

    const navItems: NavItem[] = [
        { href: "/usb", label: "Connect", requiresConnection: false },
        { href: "/settings", label: "Settings", requiresConnection: false },
        { href: "/flightlogs", label: "Logs", requiresConnection: true },
        {
            href: "/calibration",
            label: "Calibration",
            requiresConnection: true,
        },
        {
            href: "/configuration",
            label: "Configuration",
            requiresConnection: true,
        },
    ];

    $: isConnected = $connectionStore.is_connected;
    $: isDarkMode = $settingsStore.dark_mode;

    $: visibleNavItems = navItems.filter(
        (item) => !item.requiresConnection || isConnected,
    );

    $: {
        const currentPath = $page.url.pathname;
        const isCurrentPageAllowed = visibleNavItems.some(
            (item) => item.href === currentPath,
        );
        if (!isCurrentPageAllowed) {
            goto("/usb");
        }
    }

    afterUpdate(() => {
        if (isDarkMode) {
            document.documentElement.setAttribute("data-theme", "dark");
        } else {
            document.documentElement.setAttribute("data-theme", "light");
        }
    });

    onMount(() => {
        let unlisten: () => void;

        const setupListener = async () => {
            unlisten = await listen("usb-disconnected", () => {
                connectionStore.updateStatus();
            });
        };

        setupListener();

        return () => {
            if (unlisten) {
                unlisten();
            }
        };
    });
</script>

<main class="container">
    <nav role="group">
        {#each navItems as item}
            {#if !item.requiresConnection || isConnected}
                <button class="outline" on:click={() => goto(item.href)}
                    >{item.label}</button
                >
            {:else}
                <button disabled class="outline">{item.label}</button>
            {/if}
        {/each}
    </nav>
    <slot />
</main>

<style>
    nav {
        display: flex;
        padding-top: var(--pico-spacing);
        gap: var(--pico-spacing);
    }
</style>

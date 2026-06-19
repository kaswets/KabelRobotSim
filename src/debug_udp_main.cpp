// ============================================================
//  GRID 2 UDP DEBUG VIEWER
//  Doel: zie WAT er binnenkomt, voordat we de echte simulator
//  koppelen. Dit is een apart, simpel console-programma.
//
//  Compileer dit ALS EERSTE, los van de hoofd-simulator, om de
//  juiste byte-offsets te vinden voor surge/sway/heave/roll/pitch/yaw.
// ============================================================

#include "udp_telemetry.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

int main()
{
    std::cout << "========================================\n";
    std::cout << "  GRID 2 UDP DEBUG VIEWER\n";
    std::cout << "========================================\n";
    std::cout << "Zorg dat in GRID 2 / hardware_settings_config.xml staat:\n";
    std::cout << "  <udp enabled=\"true\" extradata=\"3\" ip=\"127.0.0.1\" port=\"20777\" delay=\"1\" />\n\n";

    UdpTelemetryListener listener;
    if (!listener.start(20777))
    {
        std::cout << "FOUT: kon niet starten op poort 20777.\n";
        std::cout << "Mogelijk al in gebruik door SimHub of een ander programma.\n";
        std::cin.get();
        return 1;
    }

    std::cout << "Luistert op poort 20777. Start nu GRID 2 en rij een rondje...\n";
    std::cout << "Druk Ctrl+C om te stoppen.\n\n";

    int lastCount = 0;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        auto raw = listener.getLastRawBytes();
        auto data = listener.getLatest();

        if (data.packetCount > lastCount)
        {
            lastCount = data.packetCount;

            std::cout << "--- Packet #" << data.packetCount
                      << " (" << data.lastPacketSize << " bytes) ---\n";

            // Toon als hex
            std::cout << "Hex: ";
            for (size_t i = 0; i < raw.size() && i < 64; i++)
            {
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << (int)raw[i] << " ";
            }
            std::cout << std::dec << "\n";

            // Toon als floats (elke 4 bytes)
            std::cout << "Floats: ";
            for (size_t i = 0; i + 4 <= raw.size() && i < 80; i += 4)
            {
                float val;
                std::memcpy(&val, raw.data() + i, sizeof(float));
                std::cout << "[" << i << "]" << std::fixed << std::setprecision(3)
                          << val << "  ";
            }
            std::cout << "\n\n";
        }
    }

    listener.stop();
    return 0;
}

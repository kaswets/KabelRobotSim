#pragma once
// ============================================================
//  GRID 2 UDP TELEMETRIE ONTVANGER
//  Gebruikt Winsock (ingebouwd in Windows) - geen Node.js nodig
//  Draait in dezelfde .exe als de 3D simulator
// ============================================================

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#endif

#include <cstdint>
#include <cstring>
#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>

struct TelemetryData {
    float surge = 0;   // voor/achter
    float sway  = 0;   // links/rechts
    float heave = 0;   // omhoog/omlaag
    float roll  = 0;   // rotatie X
    float pitch = 0;   // rotatie Y
    float yaw   = 0;   // rotatie Z
    bool  valid = false;
    int   packetCount = 0;
    int   lastPacketSize = 0;
};

class UdpTelemetryListener
{
public:
    UdpTelemetryListener() = default;
    ~UdpTelemetryListener() { stop(); }

    bool start(int port = 20777)
    {
#ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) return false;

        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock == INVALID_SOCKET) { WSACleanup(); return false; }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons((u_short)port);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(sock, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
            closesocket(sock);
            WSACleanup();
            return false;
        }

        // Non-blocking maken zodat de hoofd-loop niet vastloopt
        u_long mode = 1;
        ioctlsocket(sock, FIONBIO, &mode);

        running = true;
        listenThread = std::thread(&UdpTelemetryListener::listenLoop, this);
        return true;
#else
        return false;
#endif
    }

    void stop()
    {
        running = false;
        if (listenThread.joinable()) listenThread.join();
#ifdef _WIN32
        if (sock != INVALID_SOCKET) { closesocket(sock); sock = INVALID_SOCKET; }
        WSACleanup();
#endif
    }

    // Thread-safe ophalen van laatste data
    TelemetryData getLatest()
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        return latest;
    }

    // Laatste ruwe bytes voor debug weergave
    std::vector<uint8_t> getLastRawBytes()
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        return lastRaw;
    }

private:
#ifdef _WIN32
    SOCKET sock = INVALID_SOCKET;
#endif
    std::thread listenThread;
    std::atomic<bool> running{false};
    std::mutex dataMutex;
    TelemetryData latest;
    std::vector<uint8_t> lastRaw;

    void listenLoop()
    {
#ifdef _WIN32
        char buffer[2048];
        while (running)
        {
            int len = recv(sock, buffer, sizeof(buffer), 0);
            if (len > 0)
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                latest.packetCount++;
                latest.lastPacketSize = len;
                lastRaw.assign(buffer, buffer + len);

                // === HIER PARSEN WE DE GRID 2 DATA ===
                // GRID 2 / Codemasters EGO-engine UDP pakket
                // extradata="3" formaat: floats vanaf bepaalde offset
                // We moeten dit kalibreren met echte data (zie debug mode)
                parsePacket(buffer, len);
            }
            else
            {
                // Geen data binnen, korte pauze om CPU te sparen
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
#endif
    }

    float readFloatLE(const char* buf, int offset)
    {
        float val;
        std::memcpy(&val, buf + offset, sizeof(float));
        return val;
    }

    void parsePacket(const char* buf, int len)
    {
        // PLACEHOLDER PARSER - moet gekalibreerd worden met echte GRID 2 data
        // Gebruik debug mode om te zien welke offsets kloppen, pas dan hier aan.
        //
        // Typisch EGO-engine formaat (zoals DIRT/GRID series) heeft op vaste
        // offsets: speed, rpm, positie xyz, snelheid xyz, en rotatie data.
        // Onderstaande offsets zijn een eerste schatting - NIET GEVALIDEERD.

        if (len >= 60) {
            latest.valid = true;
            // Deze offsets zijn gokjes - aanpassen na debug sessie
            latest.roll  = readFloatLE(buf, 8)  / 3.14159f;  // genormaliseerd -1..1
            latest.pitch = readFloatLE(buf, 12) / 3.14159f;
            latest.yaw   = readFloatLE(buf, 16) / 3.14159f;
            latest.surge = readFloatLE(buf, 20) / 50.0f;
            latest.sway  = readFloatLE(buf, 24) / 50.0f;
            latest.heave = readFloatLE(buf, 28) / 50.0f;
        }
    }
};

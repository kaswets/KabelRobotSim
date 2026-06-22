#include "robot.h"
#include "udp_telemetry.h"
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <sstream>
#include <iomanip>
#include <string>
#include <cmath>
#include <vector>
#include <array>
#include <algorithm>

// ============================================================
//  3D Projectie helpers
// ============================================================
struct Vec3 { float x, y, z; };

Vec3 rotateView(Vec3 p, float angleX, float angleY)
{
    // Roteer om Y-as
    float cx = cosf(angleY), sx = sinf(angleY);
    float x2 = p.x * cx + p.z * sx;
    float z2 = -p.x * sx + p.z * cx;
    p.x = x2; p.z = z2;
    // Roteer om X-as
    float cy = cosf(angleX), sy = sinf(angleX);
    float y2 = p.y * cy - p.z * sy;
    float z3 = p.y * sy + p.z * cy;
    p.y = y2; p.z = z3;
    return p;
}

bool g_usePerspective = false;

sf::Vector2f project(Vec3 p, float scale, sf::Vector2f center, float camDist = 1200.f)
{
    if (g_usePerspective) {
        float persp = camDist / (camDist - p.z);
        return { center.x + p.x * scale * persp,
                 center.y - p.y * scale * persp };
    }
    else {
        // Orthografisch: geen diepte-vertekening, alle lijnen parallel
        return { center.x + p.x * scale,
                 center.y - p.y * scale };
    }
}

void drawLine3D(sf::RenderTarget& rt, Vec3 a, Vec3 b,
    float aX, float aY, float scale, sf::Vector2f center,
    sf::Color col, float thickness = 1.5f)
{
    a = rotateView(a, aX, aY);
    b = rotateView(b, aX, aY);
    auto p1 = project(a, scale, center);
    auto p2 = project(b, scale, center);
    sf::Vertex line[2] = {
        {p1, col},
        {p2, col}
    };
    rt.draw(line, 2, sf::Lines);
}

sf::Vector2f get2D(Vec3 p, float aX, float aY, float scale, sf::Vector2f center)
{
    p = rotateView(p, aX, aY);
    return project(p, scale, center);
}

// ============================================================
//  Kleuren per motor (hoog contrast, goed te onderscheiden)
// ============================================================
const sf::Color MOTOR_COLORS[8] = {
    {226, 75,  74},   // M1 rood
    {55, 138, 221},   // M2 blauw
    {99, 187, 34},    // M3 fel groen
    {239,159, 39},    // M4 oranje
    {212, 83,170},    // M5 magenta/roze
    {20, 200,180},    // M6 cyaan-teal
    {150,100, 255},   // M7 paars/lila
    {245,220, 40},    // M8 geel
};

// ============================================================
//  Sync analyse - vergelijk oud vs nieuw
// ============================================================
struct SyncResult {
    long  wanted[8];
    long  actual[8];
    long  move[8];
    int   pctOud[8];   // originele bug
    int   pctNieuw[8]; // gecorrigeerde versie
    long  sorted[8];   // gesorteerde volgorde
    int   sortIdx[8];
};

SyncResult analyseerSync()
{
    SyncResult r;
    r.wanted[0] = Mot1WantedLength; r.actual[0] = Mot1ActualLength;
    r.wanted[1] = Mot2WantedLength; r.actual[1] = Mot2ActualLength;
    r.wanted[2] = Mot3WantedLength; r.actual[2] = Mot3ActualLength;
    r.wanted[3] = Mot4WantedLength; r.actual[3] = Mot4ActualLength;
    r.wanted[4] = Mot5WantedLength; r.actual[4] = Mot5ActualLength;
    r.wanted[5] = Mot6WantedLength; r.actual[5] = Mot6ActualLength;
    r.wanted[6] = Mot7WantedLength; r.actual[6] = Mot7ActualLength;
    r.wanted[7] = Mot8WantedLength; r.actual[7] = Mot8ActualLength;

    for (int i = 0; i < 8; i++) {
        r.move[i] = abs(r.wanted[i] - r.actual[i]);
        r.sorted[i] = r.move[i];
        r.sortIdx[i] = i;
    }

    // Bubble sort (zelfde als STM32)
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7 - i; j++)
            if (r.sorted[j] < r.sorted[j + 1]) {
                std::swap(r.sorted[j], r.sorted[j + 1]);
                std::swap(r.sortIdx[j], r.sortIdx[j + 1]);
            }

    float pct = (r.sorted[0] > 0) ? 100.0f / r.sorted[0] : 0;

    // OUD: TotalMove na sort toewijzen op volgorde (de bug)
    for (int i = 0; i < 8; i++)
        r.pctOud[i] = (int)(pct * r.sorted[i] + 0.5f);

    // NIEUW: originele move per motor gebruiken
    for (int i = 0; i < 8; i++)
        r.pctNieuw[i] = (int)(pct * r.move[i] + 0.5f);

    return r;
}

// ============================================================
//  Tekst helper
// ============================================================
void drawText(sf::RenderTarget& rt, sf::Font& font, const std::string& s,
    float x, float y, int size, sf::Color col, bool bold = false)
{
    sf::Text t;
    t.setFont(font);
    t.setString(s);
    t.setCharacterSize(size);
    t.setFillColor(col);
    t.setStyle(bold ? sf::Text::Bold : sf::Text::Regular);
    t.setPosition(x, y);
    rt.draw(t);
}

// ============================================================
//  MAIN
// ============================================================
int main()
{
    const int W = 1280, H = 800;
    sf::RenderWindow window(sf::VideoMode(W, H), "Kabel Robot Simulator", sf::Style::Default);
    window.setFramerateLimit(60);
    window.setKeyRepeatEnabled(false);

    sf::Font font;
    // Probeer systeem fonts
    if (!font.loadFromFile("C:/Windows/Fonts/consola.ttf"))
        if (!font.loadFromFile("C:/Windows/Fonts/arial.ttf"))
            if (!font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"))
                font.loadFromFile("/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf");

    RobotInit();

    // GRID 2 UDP telemetrie
    UdpTelemetryListener udpListener;
    bool udpActive = false;
    float scaleSurge = 40.f, scaleSway = 30.f, scaleHeave = 35.f;
    float scaleRoll = 20.f, scalePitch = 20.f, scaleYaw = 15.f;

    // Camera - zachte hoek, minimaliseert perspectief-vertekening op verticale balken
    float camAngleX = 0.15f, camAngleY = 0.3f;
    float camScale = 0.50f;
    sf::Vector2f view3DCenter(380, 340);
    bool  dragging = false;
    sf::Vector2i lastMouse;

    // Input
    std::string inputBuf;
    std::vector<std::string> log;
    auto addLog = [&](const std::string& s) { log.push_back(s); if (log.size() > 18) log.erase(log.begin()); };

    addLog("Kabel Robot Simulator gestart");
    addLog("Commando's: MainX/Y/Z <mm>, RotX/Y/Z <gr>");
    addLog("            Center, ToRun, ToCal");
    addLog("Muis: sleep 3D view, scroll = zoom");
    addLog("F1 = GRID 2 koppelen, F2 = perspectief/orthografisch");

    while (window.isOpen())
    {
        sf::Event ev;
        while (window.pollEvent(ev))
        {
            if (ev.type == sf::Event::Closed) window.close();

            // Muis rotatie
            if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Left) {
                if (ev.mouseButton.x < 760) { dragging = true; lastMouse = { ev.mouseButton.x,ev.mouseButton.y }; }
            }
            if (ev.type == sf::Event::MouseButtonReleased) dragging = false;
            if (ev.type == sf::Event::MouseMoved && dragging) {
                camAngleY += (ev.mouseMove.x - lastMouse.x) * 0.01f;
                camAngleX += (ev.mouseMove.y - lastMouse.y) * 0.01f;
                camAngleX = std::max(-1.3f, std::min(1.3f, camAngleX));
                lastMouse = { ev.mouseMove.x, ev.mouseMove.y };
            }
            if (ev.type == sf::Event::MouseWheelScrolled)
                camScale *= (ev.mouseWheelScroll.delta > 0) ? 1.08f : 0.92f;

            // Toetsenbord input
            if (ev.type == sf::Event::TextEntered) {
                if (ev.text.unicode == '\r' || ev.text.unicode == '\n') {
                    if (!inputBuf.empty()) {
                        addLog("> " + inputBuf);
                        ParseCommand(inputBuf);
                        RobotLoop();
                        inputBuf.clear();
                    }
                }
                else if (ev.text.unicode == 8 && !inputBuf.empty()) {
                    inputBuf.pop_back();
                }
                else if (ev.text.unicode >= 32 && ev.text.unicode < 127) {
                    inputBuf += (char)ev.text.unicode;
                }
            }

            // F1 toggelt GRID 2 UDP verbinding
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::F1) {
                if (!udpActive) {
                    if (udpListener.start(20777)) {
                        udpActive = true;
                        addLog("GRID 2 UDP listener gestart op poort 20777");
                    }
                    else {
                        addLog("FOUT: kon UDP poort 20777 niet openen (al in gebruik?)");
                    }
                }
                else {
                    udpListener.stop();
                    udpActive = false;
                    addLog("GRID 2 UDP listener gestopt");
                }
            }

            // F2 toggelt orthografisch/perspectief
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::F2) {
                g_usePerspective = !g_usePerspective;
                addLog(g_usePerspective ? "Weergave: Perspectief" : "Weergave: Orthografisch");
            }

            // F3 springt naar recht voor-aanzicht (X=links/rechts, hoogte=boven/onder)
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::F3) {
                camAngleX = 0.0f; camAngleY = 0.0f;
                addLog("Camera: voor-aanzicht (M1-4 boven, M5-8 onder)");
            }
            // F4 springt naar recht zij-aanzicht
            if (ev.type == sf::Event::KeyPressed && ev.key.code == sf::Keyboard::F4) {
                camAngleX = 0.0f; camAngleY = 1.5708f;
                addLog("Camera: zij-aanzicht");
            }
        }

        // Verwerk binnenkomende GRID 2 telemetrie elke frame
        if (udpActive) {
            TelemetryData td = udpListener.getLatest();
            if (td.valid) {
                WantedMainX = td.surge * scaleSurge;
                WantedMainY = td.sway * scaleSway;
                WantedMainZ = td.heave * scaleHeave;
                WantedRotX = td.roll * scaleRoll;
                WantedRotY = td.pitch * scalePitch;
                WantedRotZ = td.yaw * scaleYaw;
                RobotLoop();
            }
        }

        window.clear(sf::Color(28, 28, 28));

        // ====================================================
        //  3D TEKENING  (links)
        // ====================================================
        sf::RectangleShape bg3d({ 760,H * 1.f });
        bg3d.setFillColor({ 35,35,40 });
        window.draw(bg3d);

        // Frame hoeken - Y/Z gewisseld zodat robot-Z (hoogte) = scherm-omhoog,
        // exact zoals de bewezen werkende HTML-versie (tv(x,y,z) = (x,z,y))
        Vec3 FC[8] = {
            {-LengthFrame / 2,  HighFrame / 2, -WidhtFrame / 2},
            {-LengthFrame / 2,  HighFrame / 2,  WidhtFrame / 2},
            { LengthFrame / 2,  HighFrame / 2, -WidhtFrame / 2},
            { LengthFrame / 2,  HighFrame / 2,  WidhtFrame / 2},
            {-LengthFrame / 2, -HighFrame / 2, -WidhtFrame / 2},
            {-LengthFrame / 2, -HighFrame / 2,  WidhtFrame / 2},
            { LengthFrame / 2, -HighFrame / 2, -WidhtFrame / 2},
            { LengthFrame / 2, -HighFrame / 2,  WidhtFrame / 2},
        };

        // Platform hoeken (haal uit ObjectCorner) - zelfde Y/Z wissel als frame
        Vec3 PC[8];
        for (int i = 0; i < 8; i++)
            PC[i] = { ObjectCorner[i + 1].X, ObjectCorner[i + 1].Z, ObjectCorner[i + 1].Y };

        sf::Color frameCol(140, 140, 130);
        sf::Color frameEdge(80, 80, 75);
        int fEdges[12][2] = { {0,1},{2,3},{4,5},{6,7},{0,2},{1,3},{4,6},{5,7},{0,4},{1,5},{2,6},{3,7} };
        for (auto& e : fEdges)
            drawLine3D(window, FC[e[0]], FC[e[1]], camAngleX, camAngleY, camScale, view3DCenter, frameCol, 1.5f);

        // Frame hoek bolletjes + labels
        for (int i = 0; i < 8; i++) {
            auto p = get2D(FC[i], camAngleX, camAngleY, camScale, view3DCenter);
            sf::CircleShape dot(4); dot.setFillColor({ 160,160,140 }); dot.setOrigin(4, 4); dot.setPosition(p);
            window.draw(dot);
            drawText(window, font, "M" + std::to_string(i + 1), p.x + 7, p.y - 7, 13, MOTOR_COLORS[i], true);
        }

        // Kabels
        for (int i = 0; i < 8; i++)
            drawLine3D(window, FC[i], PC[i], camAngleX, camAngleY, camScale, view3DCenter, MOTOR_COLORS[i], 2.0f);

        // Platform hoek bolletjes
        for (int i = 0; i < 8; i++) {
            auto p = get2D(PC[i], camAngleX, camAngleY, camScale, view3DCenter);
            sf::CircleShape dot(6); dot.setFillColor(MOTOR_COLORS[i]); dot.setOrigin(6, 6); dot.setPosition(p);
            window.draw(dot);
        }

        // Platform randen
        sf::Color platCol(100, 160, 220, 200);
        int platBot[4][2] = { {0,1},{1,3},{3,2},{2,0} };
        int platTop[4][2] = { {4,5},{5,7},{7,6},{6,4} };
        for (auto& e : platBot) drawLine3D(window, PC[e[0]], PC[e[1]], camAngleX, camAngleY, camScale, view3DCenter, platCol, 1.5f);
        for (auto& e : platTop) drawLine3D(window, PC[e[0]], PC[e[1]], camAngleX, camAngleY, camScale, view3DCenter, platCol, 1.5f);
        for (int i = 0; i < 4; i++) drawLine3D(window, PC[i], PC[i + 4], camAngleX, camAngleY, camScale, view3DCenter, platCol, 1.5f);

        // Positie label
        std::ostringstream pos;
        pos << "Pos: X=" << (int)ActualMainX << " Y=" << (int)ActualMainY << " Z=" << (int)ActualMainZ
            << "  Rot: " << (int)ActualRotX << "/" << (int)ActualRotY << "/" << (int)ActualRotZ;
        drawText(window, font, pos.str(), 10, 10, 14, { 180,180,180 });
        drawText(window, font, Inpos ? "IN POSITIE" : "BEWEEGT...", 10, 28, 14,
            Inpos ? sf::Color(100, 220, 80) : sf::Color(220, 160, 60), true);

        // GRID 2 UDP status
        std::string udpStatus = udpActive
            ? ("GRID 2: VERBONDEN  (pakketten: " + std::to_string(udpListener.getLatest().packetCount) + ")  [F1 = stop]")
            : "GRID 2: niet verbonden  [F1 = start luisteren op poort 20777]";
        drawText(window, font, udpStatus, 10, 46, 13,
            udpActive ? sf::Color(100, 200, 255) : sf::Color(120, 120, 120));

        // Projectie-modus
        std::string projStatus = g_usePerspective
            ? "Weergave: PERSPECTIEF  [F2 = wissel naar Orthografisch]"
            : "Weergave: ORTHOGRAFISCH  [F2 = wissel naar Perspectief]";
        drawText(window, font, projStatus, 10, 64, 13, sf::Color(180, 180, 160));

        // ====================================================
        //  RECHTER PANEEL - sync analyse + log
        // ====================================================
        float rx = 770;
        drawText(window, font, "MOTOR SYNC ANALYSE", rx, 10, 16, { 220,220,200 }, true);
        drawText(window, font, "(na bewegingscommando)", rx, 30, 12, { 140,140,130 });

        auto sr = analyseerSync();

        // Header
        drawText(window, font, "Motor  Stappen  Procent(oud)  Procent(fix)  Status", rx, 55, 13, { 160,160,150 });

        for (int i = 0; i < 8; i++) {
            float y = 75 + i * 32;
            // Kleurstip
            sf::RectangleShape bar({ 8,20 }); bar.setFillColor(MOTOR_COLORS[i]); bar.setPosition(rx, y + 2); window.draw(bar);

            std::ostringstream s;
            s << "M" << (i + 1) << "  ";
            s << std::setw(7) << sr.move[i] << "  ";
            s << std::setw(10) << sr.pctOud[i] << "%  ";
            s << std::setw(10) << sr.pctNieuw[i] << "%  ";

            bool mismatch = (sr.pctOud[i] != sr.pctNieuw[i]);
            sf::Color col = mismatch ? sf::Color(230, 100, 80) : sf::Color(100, 210, 80);
            drawText(window, font, s.str(), rx + 14, y, 14, col);

            if (mismatch) {
                int diff = sr.pctNieuw[i] - sr.pctOud[i];
                std::string mark = " BUG! oud=" + std::to_string(sr.pctOud[i]) + "% moet=" + std::to_string(sr.pctNieuw[i]) + "%";
                drawText(window, font, mark, rx + 340, y, 12, { 230,130,60 });
            }
            else if (sr.move[i] == 0) {
                drawText(window, font, " stilstand", rx + 340, y, 12, { 100,100,100 });
            }
        }

        // Separator
        sf::RectangleShape sep({ 500,1 }); sep.setFillColor({ 70,70,65 }); sep.setPosition(rx, 340); window.draw(sep);

        // Gesorteerde volgorde uitleg
        drawText(window, font, "Sort volgorde (langste eerst):", rx, 350, 13, { 160,160,150 });
        std::string sortStr = "";
        for (int i = 0; i < 8; i++) {
            sortStr += "M" + std::to_string(sr.sortIdx[i] + 1) + "(" + std::to_string(sr.sorted[i]) + ") ";
            if (i == 3) sortStr += "\n";
        }
        drawText(window, font, sortStr, rx, 368, 12, { 140,180,140 });

        // Separator
        sep.setPosition(rx, 420); window.draw(sep);

        // Log venster
        drawText(window, font, "CONSOLE", rx, 428, 14, { 180,180,160 }, true);
        for (int i = 0; i < (int)log.size(); i++)
            drawText(window, font, log[i], rx, 448 + i * 17, 12,
                log[i][0] == '>' ? sf::Color(120, 200, 255) : sf::Color(160, 160, 150));

        // Input box
        sf::RectangleShape inputBox({ 500,26 }); inputBox.setFillColor({ 45,45,50 });
        inputBox.setOutlineColor({ 80,80,90 }); inputBox.setOutlineThickness(1);
        inputBox.setPosition(rx, 758); window.draw(inputBox);
        drawText(window, font, "> " + inputBuf + "_", rx + 4, 761, 13, { 200,220,255 });

        window.display();
    }
    return 0;
}
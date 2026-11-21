import hypermedia.net.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

ToxiclibsSupport gfx;

// --- UDP Objects & IPs ---
UDP udpTeapot;                       // Listens to plane telemetry on port 4210 / sends Head Tracking
UDP udpHeadTrack;                    // Listens to Head Tracker IMU stream on port 4000
String espIP = "192.168.137.93";     // ESP8266 IP address
int espPort = 4210;                  // Target UDP port on ESP8266

// --- Teapot & Telemetry Variables ---
byte[] teapotPacket = new byte[14];  //
int serialCount = 0;                //
int synced = 0;                     //

float[] q = {1, 0, 0, 0};           // w, x, y, z  (identity by default)
Quaternion quat = new Quaternion(1, 0, 0, 0); //

String extraMessage = "";           //
String localBuffer = "";            //
int localSpeed = 0;                 //
int localSignal = 0;                //

// --- Plane attitude/heading (derived from the teapot quaternion) ---
float planeRoll = 0;                 // Bank angle, degrees
float planePitch = 0;                // Nose up/down, degrees
float planeYaw = 0;                  // Raw yaw from the sensor, degrees
float headingOffset = 0;             // Yaw value captured at first packet -> treated as North (0)
boolean headingInit = false;         // Have we captured the initial heading yet?

// --- Diagnostics: are '$' quaternion packets actually arriving on port 4210? ---
int quatPacketCount = 0;
long lastQuatPacketTime = -1;

// --- Head Tracking Variables (still computed & sent to the ESP, just not drawn anymore) ---
float headPitch = 0;                // Look Up / Down
float headYaw   = 0;                // Look Left / Right
long lastHeadTime = 0;               //
String rawHeadData = "Waiting for data..."; //
long lastSendToEspTime = 0;          // Rate limiter for sending to ESP

// --- HUD helper graphics ---
PGraphics attitudePG;   // Offscreen buffer used to render the artificial horizon
PImage circleMaskImg;   // Circular alpha mask so the horizon renders as a round gauge
PFont hudFont;          // Clean sans-serif font for the compass ribbon (DJI-style)

void setup() {
    size(800, 600, OPENGL);
    surface.setResizable(true); // let the window be resized; layout below adapts every frame
    surface.setLocation((displayWidth - width) / 2, (displayHeight - height) / 2);
    gfx = new ToxiclibsSupport(this);

    lights();
    smooth();

    // 1. Setup UDP sockets
    udpTeapot = new UDP(this, 4210);
    udpTeapot.setReceiveHandler("receiveTeapotData"); // Bind dedicated handler
    udpTeapot.listen(true);

    udpHeadTrack = new UDP(this, 4000);
    udpHeadTrack.setReceiveHandler("receiveHeadData"); // Bind dedicated handler
    udpHeadTrack.listen(true);

    lastHeadTime = millis();

    // 2. Build a circular mask once, used to clip the attitude indicator into a round dial
    attitudePG = createGraphics(220, 220);
    PGraphics maskPG = createGraphics(220, 220);
    maskPG.beginDraw();
    maskPG.background(0);
    maskPG.noStroke();
    maskPG.fill(255);
    maskPG.ellipse(110, 110, 220, 220);
    maskPG.endDraw();
    circleMaskImg = maskPG.get();

    hudFont = createFont("Arial", 32);

    println("[DEBUG] Listening on 4210 (Teapot/Plane) & 4000 (Head Tracker)...");
}

// --- DEDICATED HANDLER 1: PORT 4000 (HEAD TRACKER) ---
void receiveHeadData(byte[] data, String ip, int port) {
    rawHeadData = new String(data).trim();
    String[] values = splitTokens(rawHeadData, " ,:\n\r");

    if (values.length >= 6) {
        float ax = float(values[0]);
        float ay = float(values[1]);
        float az = float(values[2]);
        float gx = float(values[3]);

        long currentTime = millis();
        float dt = (currentTime - lastHeadTime) / 1000.0;
        lastHeadTime = currentTime;

        headPitch = degrees(atan2(az, -ax));
        if (abs(gx) > 0.05) {
            headYaw += degrees(gx) * dt;
        }
    }
}

// --- DEDICATED HANDLER 2: PORT 4210 (PLANE TEAPOT & TELEMETRY) ---
void receiveTeapotData(byte[] data, String ip, int port) {
    for (int i = 0; i < data.length; i++) {
        int ch = data[i] & 0xFF;

        if (ch == '$') {
            synced = 1;
            serialCount = 0;
            extraMessage = "";
        } else if (ch == '#') {
            synced = 2;
            localBuffer = "";
        } else if (synced == 1) {
            if (serialCount < 13) {
                teapotPacket[serialCount++] = (byte)ch;
            } else {
                if (ch == '\n' || ch == '\r') {
                    synced = 0;
                } else {
                    if ((ch >= '0' && ch <= '9') || ch == '.') {
                        extraMessage += (char)ch;
                    }
                }

                if (serialCount == 13) {
                    q[0] = ((teapotPacket[1] << 8) | (teapotPacket[2] & 0xff)) / 16384.0f;
                    q[1] = ((teapotPacket[3] << 8) | (teapotPacket[4] & 0xff)) / 16384.0f;
                    q[2] = ((teapotPacket[5] << 8) | (teapotPacket[6] & 0xff)) / 16384.0f;
                    q[3] = ((teapotPacket[7] << 8) | (teapotPacket[8] & 0xff)) / 16384.0f;
                    for (int j = 0; j < 4; j++) if (q[j] >= 2) q[j] = -4 + q[j];
                    quat.set(q[0], q[1], q[2], q[3]);
                    serialCount++;

                    quatPacketCount++;
                    lastQuatPacketTime = millis();

                    // Update plane attitude/heading from the fresh quaternion
                    float[] eul = quatToEuler(q[0], q[1], q[2], q[3]);
                    planeRoll  = eul[0];
                    planePitch = eul[1];
                    planeYaw   = eul[2];

                    // Latch the very first heading we see as "North" (0 deg)
                    if (!headingInit) {
                        headingOffset = planeYaw;
                        headingInit = true;
                    }
                }
            }
        } else if (synced == 2) {
            if (ch == '\n' || ch == '\r') {
                String[] list = split(localBuffer, ',');
                if (list.length >= 2) {
                    localSpeed = int(list[0]);
                    localSignal = int(list[1]);
                }
                synced = 0;
            } else {
                localBuffer += (char)ch;
            }
        }
    }
}

// Convert a quaternion (w,x,y,z) to roll/pitch/yaw in degrees
float[] quatToEuler(float w, float x, float y, float z) {
    float sinrCosp = 2 * (w * x + y * z);
    float cosrCosp = 1 - 2 * (x * x + y * y);
    float roll = degrees(atan2(sinrCosp, cosrCosp));

    float sinp = 2 * (w * y - z * x);
    sinp = constrain(sinp, -1, 1);
    float pitch = degrees(asin(sinp));

    float sinyCosp = 2 * (w * z + x * y);
    float cosyCosp = 1 - 2 * (y * y + z * z);
    float yaw = degrees(atan2(sinyCosp, cosyCosp));

    return new float[] { roll, pitch, yaw };
}

void draw() {
    background(15);

    // Base scale factor so gauges/text stay proportional at any window size (800x600 = 1.0)
    float uiScale = height / 600.0;

    // Current heading relative to the very first reading (which we treat as North / 0 deg)
    float heading = ((planeYaw - headingOffset) % 360 + 360) % 360;

    // --- TOP-RIGHT VIEWPORT: 3D Plane Orientation (Teapot), tucked into the corner ---
    pushMatrix();
    translate(width * 0.9, height * 0.14, 0);
    scale(0.45 * uiScale);
    float[] axis = quat.toAxisAngle();
    rotate(axis[0], axis[2], axis[3], axis[1]);

    fill(255, 0, 0, 200);
    box(10, 10, 200);
    fill(0, 0, 255, 200);
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();
    fill(0, 255, 0, 200);
    beginShape(TRIANGLES);
    vertex(-100,  2, 30); vertex(0,  2, -80); vertex(100,  2, 30);
    vertex(-100, -2, 30); vertex(0, -2, -80); vertex(100, -2, 30);
    vertex(-2, 0, 98); vertex(-2, -30, 98); vertex(-2, 0, 70);
    vertex( 2, 0, 98); vertex( 2, -30, 98); vertex( 2, 0, 70);
    endShape();
    beginShape(QUADS);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex( 100, 2, 30); vertex( 100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(100, -2,  30); vertex(100, 2,  30);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2, -30, 98); vertex(-2, -30, 98);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    vertex(-2, -30, 98); vertex(2, -30, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    endShape();
    popMatrix();

    // --- 2D OVERLAY HUD ---
    hint(DISABLE_DEPTH_TEST);
    camera();
    noLights();

    // Top-left: Plane Telemetry (just the raw readings, no header/heading/attitude/diagnostics)
    fill(0, 255, 0);
    textSize(13 * uiScale);
    float tx = width * 0.02;
    float ty = height * 0.19;
    float lh = height * 0.03;
    text("Speed: " + localSpeed + " (" + int((localSpeed - 1350)/6.5) + "%)", tx, ty);
    text("Signal: " + localSignal + " dBm", tx, ty + lh);
    text("Battery: " + extraMessage + " V", tx, ty + 2*lh);

    // Bottom-left: Attitude meter (artificial horizon) - smaller than before
    float attDiameter = 135 * uiScale;
    drawAttitudeIndicator(width * 0.08, height * 0.85, attDiameter, planeRoll, planePitch, uiScale);

    // Bottom: Compass ribbon (initial heading = North / 0 deg) - smaller, no encasing box
    float compW = width * 0.45;
    float compH = 25 * uiScale;
    drawCompass((width - compW) / 2, height * 0.93, compW, compH, heading, uiScale);

    hint(ENABLE_DEPTH_TEST);

    // Transmit Head Tracking data to ESP8266 every ~50ms (20Hz)
    if (millis() - lastSendToEspTime > 50) {
        sendHeadTrackingToESP();
        lastSendToEspTime = millis();
    }
}

// --- Bottom-left artificial-horizon style attitude meter ---
void drawAttitudeIndicator(float cx, float cy, float diameter, float rollDeg, float pitchDeg, float uiScale) {
    float w = attitudePG.width;
    float h = attitudePG.height;
    float pitchPixels = pitchDeg * (h / 2.0) / 45.0; // 45 deg of pitch = half the dial height

    attitudePG.beginDraw();
    attitudePG.background(0);
    attitudePG.noStroke();

    attitudePG.pushMatrix();
    attitudePG.translate(w/2, h/2);
    attitudePG.rotate(radians(-rollDeg));
    attitudePG.translate(0, pitchPixels);

    // Sky
    attitudePG.fill(60, 140, 230);
    attitudePG.rectMode(CORNER);
    attitudePG.rect(-w*1.5, -h*3, w*3, h*3);
    // Ground
    attitudePG.fill(120, 72, 30);
    attitudePG.rect(-w*1.5, 0, w*3, h*3);
    // Horizon line
    attitudePG.stroke(255);
    attitudePG.strokeWeight(2);
    attitudePG.line(-w*1.5, 0, w*1.5, 0);

    // Pitch ladder marks every 10 degrees (+/- 30)
    attitudePG.textAlign(CENTER, CENTER);
    attitudePG.textSize(10);
    for (int p = -30; p <= 30; p += 10) {
        if (p == 0) continue;
        float ly = -p * (h / 2.0) / 45.0;
        float lineHalfWidth = (p % 20 == 0) ? 30 : 16;
        attitudePG.stroke(255);
        attitudePG.line(-lineHalfWidth, ly, lineHalfWidth, ly);
    }
    attitudePG.popMatrix();

    // Fixed aircraft reference symbol (does not rotate with roll/pitch)
    attitudePG.stroke(255, 220, 0);
    attitudePG.strokeWeight(3);
    attitudePG.line(w/2 - 35, h/2, w/2 - 10, h/2);
    attitudePG.line(w/2 + 10, h/2, w/2 + 35, h/2);
    attitudePG.noStroke();
    attitudePG.fill(255, 220, 0);
    attitudePG.ellipse(w/2, h/2, 6, 6);

    attitudePG.endDraw();
    attitudePG.mask(circleMaskImg);

    imageMode(CENTER);
    image(attitudePG, cx, cy, diameter, diameter); // scale the fixed-size buffer to the requested size
    imageMode(CORNER);

    noFill();
    stroke(255);
    strokeWeight(2 * uiScale);
    ellipse(cx, cy, diameter, diameter);
}

// --- Bottom heading ribbon, styled after the DJI FPV-goggle OSD compass tape ---
// 0 deg = the very first heading received (treated as North). No background box - just the tape itself.
void drawCompass(float x, float y, float w, float h, float heading, float uiScale) {
    float centerX = x + w/2;
    float pixelsPerDegree = w / 100.0; // ~ +/-50 deg of tape visible across the width

    clip(x, y, w, h);
    textAlign(CENTER, CENTER);
    textFont(hudFont); // clean sans for the DJI look
    for (int a = 0; a < 360; a += 15) {
        float diff = ((a - heading + 540) % 360) - 180;
        if (abs(diff) > 55) continue;

        float px = centerX + diff * pixelsPerDegree;
        boolean isCardinal = (a % 90 == 0);
        boolean isInter    = (a % 45 == 0);

        // Ticks: short/thin for the 15deg marks, taller/thicker for cardinals
        float tickLen = isCardinal ? h * 0.55 : (isInter ? h * 0.4 : h * 0.22);
        stroke(255);
        strokeWeight((isCardinal ? 2 : 1) * uiScale);
        line(px, y + h - tickLen, px, y + h - 2);

        if (isCardinal || isInter) {
            String label;
            switch (a) {
                case 0:   label = "N";  break;
                case 45:  label = "NE"; break;
                case 90:  label = "E";  break;
                case 135: label = "SE"; break;
                case 180: label = "S";  break;
                case 225: label = "SW"; break;
                case 270: label = "W";  break;
                case 315: label = "NW"; break;
                default:  label = str(a);
            }
            noStroke();
            fill(isCardinal ? color(255, 210, 0) : color(255)); // N/E/S/W pop in amber like DJI
            textSize((isCardinal ? 13 : 10) * uiScale);
            text(label, px, y + h * 0.28);
        }
    }
    noClip();

    // Fixed center caret marking "current heading" (tape scrolls under it, DJI-style)
    fill(255, 210, 0);
    noStroke();
    triangle(centerX - 5*uiScale, y - 8*uiScale, centerX + 5*uiScale, y - 8*uiScale, centerX, y - 1);
    stroke(255, 210, 0);
    strokeWeight(2 * uiScale);
    line(centerX, y, centerX, y + h);

    // Boxed numeric heading readout sitting just above the ribbon, DJI-style
    String headStr = nf(heading, 3, 0) + "\u00B0";
    textFont(hudFont);
    textSize(13 * uiScale);
    float boxW = textWidth(headStr) + 16 * uiScale;
    float boxH = 18 * uiScale;
    noStroke();
    fill(0, 0, 0, 170);
    rect(centerX - boxW/2, y - 8*uiScale - boxH, boxW, boxH, 4);
    stroke(255, 210, 0);
    strokeWeight(1);
    noFill();
    rect(centerX - boxW/2, y - 8*uiScale - boxH, boxW, boxH, 4);
    fill(255, 210, 0);
    textAlign(CENTER, CENTER);
    text(headStr, centerX, y - 8*uiScale - boxH/2 + 1);
    textAlign(LEFT, BASELINE);
}

// Forward formatted head angles back to ESP8266
void sendHeadTrackingToESP() {
    String packet = "@" + nf(headPitch, 0, 2) + "," + nf(headYaw, 0, 2) + "\n";
    udpTeapot.send(packet, espIP, espPort);
}

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
        angle += angleIncrement;
    }
    endShape();

    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }

    if (bottomRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        vertex(0, tall, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
}

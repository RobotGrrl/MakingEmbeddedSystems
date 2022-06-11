/*
 * AccioRange_Speak
 * -----------------
 * Uses Apple's text to speech engine via AppleScript to
 * vocalize the incoming data - the resulting distance -
 * sent from the microcontroller via bluetooth (serial)
 *
 * Erin K - June 11, 2022
 * License: MIT
 */

import processing.serial.*;

Serial device;
int port = 99;
boolean connected = false;

String s = "";

long last_speak = 0;

void setup() {
  size(200,200);
  shellExec("osascript -e \"say \\\"Ack key yo Range\\\" \"");
  
  println("serial port list:");
  for(int i=0; i<Serial.list().length; i++) {
    print("[" + i + "] ");
    println(Serial.list()[i]); 
  }
  
  // /dev/cu.HC-05-DevB
  device = new Serial(this, Serial.list()[2], 9600);
  println("connected");
  connected = true;
}

void draw() {
 
  if(connected) {
    char c;
    while(device.available() > 0) {
      c = device.readChar();
      //print(c);
      if(c == ';') {
        processString();
      } else {
        s += c; 
      }
    }
  }
  
}

void processString() {
 
  print(s);
  if(s == "-1") return; // skip
  
  float read = -1;
  try {
    read = (float)Integer.parseInt(s);
  } catch(Exception e) {
    return;
  }
  float result_distance = read/10; // in cm
  if(read == -1) return; // skip
  
  print("........");
  println(result_distance);
  
  s = "";
  
  // execute applescript
  if(millis() < 5*1000) return; // there's a bunch of data at the start
  if(millis()-last_speak <= 1000 && last_speak > 0) return; // space it out a bit
  
  String command = ("osascript -e \"say \\\" "+ result_distance +" \\\" \"");
  shellExec(command);
  last_speak = millis();
  
}

void shellExec ( String command ) {
  try {
    Process process = Runtime.getRuntime().exec ( new String[]{ "/bin/bash", "-c", command } );
  } 
  catch(Exception e) {
    
  }
}



// https://forum.processing.org/beta/num_1201876176.html
// https://www.chem.uwec.edu/chem101/System%209Folder/Help/AppleScript%20Help/at/pgs/at81.htm
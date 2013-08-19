/*
 * Communicates with host PC over serial connection, 
 * sends the status of the switches.
 *
 * 8 digital inputs are read, corresponding to arduino 
 * inputs 2,3,4,5,6,7,8,9. 
 * These 8 inputs are written as a byte to the serial connection,
 * as "DX" where X is the char representation of the byte.
 *
 * The status is only written when it changes, i.e a drink is placed
 * or removed.
 *
 * The current state can be requested by writing "!R" to the connection,
 * causing the device to report "DX", 'X' being the byte.
 *
 * On error (eg bad request), "EN" where N is error code. 0 = bad request.
 * 
 */

int incoming_command[2];
int command_recv_pos=0;
boolean new_command = false;

int i;

int input_states, previous_input_states;

void setup() {
  Serial.begin(9600);
  
  // Set inputs to have weak pull ups
  for (i=2; i<10; i++) {
    pinMode(i, INPUT);
    digitalWrite(i, HIGH);
  }
}

void serialEvent() {
  // Receive commands over serial
  // set new_command true when command received
  while (Serial.available()) {
    incoming_command[command_recv_pos]=Serial.read();
    command_recv_pos+=1;
    if (command_recv_pos>1) {
      new_command=true;
      command_recv_pos=0;
    }
  }
}


void loop() {
    previous_input_states = input_states;
    // combine input ports parts into single byte
    input_states = ((PIND >> 2)) | ((PINB & B00000011) << 6);
    
    delay(10);
      
    if (new_command) {
      if ((incoming_command[0]=='!') && (incoming_command[1]=='R')) {
        Serial.print('D');
        Serial.print((char)input_states);
      } else {
        Serial.print('E');
        Serial.print('0');
      }
      new_command=false; 
    }
    if (input_states != previous_input_states) {
        Serial.print('D');
        Serial.print((char)input_states);
    }
}




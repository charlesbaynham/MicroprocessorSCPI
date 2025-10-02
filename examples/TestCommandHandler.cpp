
#include <Arduino.h>
#include "CommandHandler.h"

CommandHandler<25> h;

bool flag = false;

int a, b, c;

void sayHello(const ParameterLookup &params)
{
  Serial.println("Hello world!");
}

void setFlag(const ParameterLookup &params)
{
  flag = true;
}

void setInts(const ParameterLookup &params)
{
  a = atoi(params[1]);
  b = atoi(params[2]);
  c = atoi(params[3]);
}

float d, e, f;
void setFloats(const ParameterLookup &params)
{
  d = atof(params[1]);
  e = atof(params[2]);
  f = atof(params[3]);
}

void testFloats()
{
  d = 0.0;
  e = 0.0;
  f = 0.0;

  h.addCommandChar('S');
  h.addCommandChar('E');
  h.addCommandChar('T');
  h.addCommandChar('F');
  h.addCommandChar('L');
  h.addCommandChar('O');
  h.addCommandChar('A');
  h.addCommandChar('T');
  h.addCommandChar('S');
  h.addCommandChar(' ');
  h.addCommandChar('1');
  h.addCommandChar('.');
  h.addCommandChar('5');
  h.addCommandChar(' ');
  h.addCommandChar('2');
  h.addCommandChar('.');
  h.addCommandChar('5');
  h.addCommandChar(' ');
  h.addCommandChar('3');
  h.addCommandChar('.');
  h.addCommandChar('5');
  h.addCommandChar('\n');

  h.executeCommand();

  if (fabs(d - 1.5) < 1e-6 && fabs(e - 2.5) < 1e-6 && fabs(f - 3.5) < 1e-6)
  {
    Serial.println("Floats set correctly, as expected.");
  }
  else
  {
    Serial.print("Floats not set correctly, unexpected! d=");
    Serial.print(d, 8);
    Serial.print(", e=");
    Serial.print(e, 8);
    Serial.print(", f=");
    Serial.println(f, 8);
  }
}

void testHello()
{
  if (h.commandWaiting())
  {
    Serial.println("Command waiting at startup, unexpected!");
  }
  else
  {
    Serial.println("No command waiting at startup, as expected.");
  }
  h.addCommandChar('H');
  h.addCommandChar('E');
  h.addCommandChar('L');
  h.addCommandChar('L');
  h.addCommandChar('O');
  h.addCommandChar('\n');

  if (h.commandWaiting())
  {
    Serial.println("Command waiting after adding HELLO, as expected.");
  }
  else
  {
    Serial.println("No command waiting after adding HELLO, unexpected!");
  }

  h.executeCommand();
}

void testFlag()
{
  // Test flag
  flag = false;

  h.addCommandChar('F');
  h.addCommandChar('L');
  h.addCommandChar('A');
  h.addCommandChar('G');
  h.addCommandChar('\n');

  h.executeCommand();
  if (flag)
  {
    Serial.println("Flag set, as expected.");
  }
  else
  {
    Serial.println("Flag not set, unexpected!");
  }
}

void testIntegers()
{
  a = 0;
  b = 0;
  c = 0;

  h.addCommandChar('S');
  h.addCommandChar('E');
  h.addCommandChar('T');
  h.addCommandChar('I');
  h.addCommandChar('N');
  h.addCommandChar('T');
  h.addCommandChar('S');
  h.addCommandChar(' ');
  h.addCommandChar('1');
  h.addCommandChar(' ');
  h.addCommandChar('2');
  h.addCommandChar(' ');
  h.addCommandChar('3');
  h.addCommandChar('\n');

  h.executeCommand();

  if (a == 1 && b == 2 && c == 3)
  {
    Serial.println("Integers set correctly, as expected.");
  }
  else
  {
    Serial.print("Integers not set correctly, unexpected! a=");
    Serial.print(a);
    Serial.print(", b=");
    Serial.print(b);
    Serial.print(", c=");
    Serial.println(c);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting up...");

  Serial.println("Registering commands...");
  h.registerCommand(COMMANDHANDLER_HASH("HELLO"), 0, sayHello);
  h.registerCommand(COMMANDHANDLER_HASH("FLAG"), 0, setFlag);
  h.registerCommand(COMMANDHANDLER_HASH("SETINTS"), 3, setInts);
  h.registerCommand(COMMANDHANDLER_HASH("SETFLOATS"), 3, setFloats);

  Serial.println("Setup complete. Testing...");

  testHello();
  testFlag();
  testIntegers();
  testFloats();

  Serial.println("All tests complete.");
};

void loop()
{
  if (h.commandWaiting())
  {
    // Execute first waiting command
    CommandHandlerReturn result = h.executeCommand();

    // Report any errors
    switch (result)
    {
    case CommandHandlerReturn::NO_ERROR:
      // No error, so nothing to report
      break;

    case CommandHandlerReturn::COMMAND_NOT_FOUND:
      Serial.println(F("Command not found"));
      break;

    case CommandHandlerReturn::WRONG_NUM_OF_PARAMS:

      Serial.println(F("Wrong number of params"));
      break;

    default:
      Serial.print(F("Error code "));
      Serial.println(int(result));
    }
  }

  // Check for serial input
  while (Serial.available())
  {
    // Queue input for processing
    h.addCommandChar(Serial.read());
  }
}

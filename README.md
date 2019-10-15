
Microprocessor SCPI Command Parser
==================================

## 2nd October 2017 - Charles Baynham

This library is designed to receive serial input on a microprocessor, parse
it, match it against a list of possible commands and call the appropriate
one. It incorporates basic parameter checking (correct number only) and
returns a `CommandHandlerReturn` object which indicates how each call went.

It is designed to be lightweight in terms of RAM usage: space requirements
are 6 bytes + 8 per command + buffer size, where the buffer should be the
length of your longest possible command (default `COMMAND_SIZE_MAX = 128`
bytes).

See the example files for a demonstration of how to use the library.

Briefly, to use this library, first create a `CommandHander` object:

	// Create a CommandHandler object to hold 5 commands
	CommandHandler<5> h;

Then, create functions that match the template of a `commandFunction`. In
other words, the must return void and take only a const ParameterLookup&
object as a paremeter. This object contains c strings for each of the
parameters passed to your function by the user.

For example, if the user input `VOLT? 1 2.872 hello` :

    params[0]    =    "VOLT?"
    params[1]    =    "1"
    params[2]    =    "2.872"
    params[3]    =    "hello"
    params[4]    =    NULL pointer
    params[-5]   =    NULL pointer

Commands must be registered using `registerCommand()`. E.g. to register the
above command that takes 3 parameters:

	h.registerCommand(COMMANDHANDLER_HASH("volt?"), 3, &theFunctionToCall)

N.B. `h.registerCommand("volt?", 3, &theFunctionToCall)` would also work, but
would store the string in memory which is wasteful.

When serial input is received, you must pass it along to the `CommandHandler`
using `h.addCommandChar`.

To check for waiting commands use `h.commandWaiting()`.

To call any queued commands use `h.executeCommand()`.

Since this library is designed to be run on a microcontroller where memory
comes at a premium, it avoid all dynamically assigned memory: all features are
implemented statically and all variables, buffers etc. are assigned on the
stack and have a lifetime tied to the `CommandHandler` object.

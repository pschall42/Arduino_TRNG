# Arduino_TRNG
Environmental data is used to generate random numbers to an LCD display and to USB Serial output

This is an upload of some of my work from my engineering Capstone class. The LCD code is not mine, and a portion of the output processing function referring to [this website](http://graphics.stanford.edu/~seander/bithacks.html#ReverseParallel) about reversing bits in a word isn't, but the rest is my own code.

The required circuit board is an Arduino Mega, with data gathered by the following inputs: an unstable analog circuit that picks up ambient electrical noise, an unstable digital circuit made of SR latches, a microphone, two photodiode circuits (one with a single photodiode, the other using two photodiodes with the output given by a differential amplifier), and a geiger counter.

If anyone wants a more detailed explanation of how it works and the resources we used, send me a message.

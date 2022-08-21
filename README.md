# CS-350
Emerging Sys Arch &amp; Tech (Embedded Systems) Coursework on the TI CC3220S Launchpad board. 


Summarize the project and what problem it was solving.
  The milestone and the Project both focused on developing code to complete tasks in the C Language on an embedded system. In particular, the milestone focused on displaying an SOS and OK signal with the LED lights using a state machine and a timer interrupt service. In this milestone pressing one button would start a repeating SOS message that when another button was pressed would finish and then display a OK message. While the LED portion of the project may not be useful in real world application the signals those LED lights recieved could easily be changed to a radio transmitter that sent the message similar to the way the message is displayed in LEDs. As for the project the overall goal was to function as a thermostat. This project uses the embedded Thermostat with a user defined threshold to signal a light on or off depending on the temperature. A user can change this threshold with the onboard buttons and thereby simulating a thermostat. This would work for operating a furnace as a signal could be sent to the furnace to turn on and no signal to turn the furnace off. Additionally this project displays to the user (Using UART) the pertitnent stats such as temperature, threshhold, signal status and seconds since reset. 


What did you do particularly well?
  I think overall I managed to build effective and relatively efficient software to handle the related tasks. I was able to add requested functionality and I did not have to have massive inefficiencies in the software itself.


Where could you improve?
  Learning a bit more about how the boards function and how to better test issues on the board would likely be a great place to improve. Unfortunately most exceptions thrown while running code on this board are handled internally so debugging in an embedded environment is definitely an area where I could and to some extent have improved. 


What tools and/or resources are you adding to your support network?
  Personally, I think I have relatively well built support network in terms of resources though using this board with its limited documentation was a challenge, I often had to test code extensively as examples using this board an associated packages/libraries were few and far between. One thing I would like to look into is board emulation as I think an emulator for an embedded system would make the troubleshooting and debugging process alot faster overall.


What skills from this project will be particularly transferable to other projects and/or course work?
  Just learning how to work with embedded systems and program in strictly C were two skills which I lacked entirely before this course so both of these skills will likely be transferable into anything I do. The knowledge and understanding I gained throughout this course in both of these areas are directly transferable to future project and future work in the field. When discussing with my peers from other schools it seems that most of them have had little to no exposure to either of these areas so I definitely think this is helpful (I know more than a few CS majors that have never even learned C++, let alone C or embedded system technology.)


How did you make this project maintainable, readable, and adaptable?
  I try to overly comment my code to ensure that anyone looking at it can easily read the comments and understand what is happening, even if they are not familiar with the libraries involved or the language that is being used (though I do assume some level of basic programming knowledge). In this way it becomes easier for another person to pickup my code and modify it without much knowledge related to it or its function (again assuming a base level of CS understanding). 


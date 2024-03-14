# GRAM's Music Synthesizer
## Key press detection
  ### Decoupled key scanning
  A basic key scanning function would always detect the pressed keys starting from the lowest position to the highest (0-12). This is okay if the sound waveform from each key is not changing over time. But with the
  decay function, the maximum amplitude of sound wave of each key is decreasing over time and the decay is independent to each key.
  
  This would create the issue of keys being desynchronised (see section 2). To solve this issue, when a key is released, it must remain at the same position as before instead of moving to a lower position. In other words, the keys must be detected in the order of their presses, not in the order of their position.
  
  First, a Boolean array of length 12 is used in place of the input variable. For every key scanning execution, the key that’s being pressed is registered true in corresponding entry in the array and this key will get skipped in the next scanning iteration. There are 3 cases that can happen:
  1.	A key was previously not being pressed and is now being pressed. In this case, the entry in the array corresponding to that key is set true and not will increase by 1.
  2.	A key was previously being presses and is now released. In this case we used a std::find function to location the index in the array corresponding to the released key. The entry at that index will get set false. And nok will decrease by 1.
  3.	A key was not pressed before and remains undressed or was pressed and remains being presses. No nothing in this case.

## Display
## User interface
## Instrument waveform generation
  ### Instrument look-up-table (LUT)
  Commercial synthesizers stores sound waveforms in their internal memory. This is because a lot of the real instrument sounds cannot be easily generated using mathematical functions. A typical approach is to record the sound of instruments playing, and then extract a section of it then play it repeatedly.

To do this, a waveform_lut is created. It’s a IxfxT tensor where I is the number of instruments, f is the number of frequencies (in this case 12) and T is the number of entries in a full period of that frequency. 

Inside the double buffer writing task, Vout is calculated by reading the entries in waveform_lut by indexing with instru, tone_idx, t and Ts. The summed amplitude from multiple frequencies will be divided by the number of keys (nok) to ensure it’s between 0 and 4095.

## Audio
  ### Tone decay
  Some Instruments has the property of producing sound whose amplitude decreases overtime. To replicate this effect in a digital synthesiser, Vout need to decrease over time.
  ### Beat function
  The synthesizer also has the function of using the keyboard as a beat generator.
The difference between the waveform sound and a beat sound is that, an waveform is periodic whereas a beat is typically non-periodic and has a varying frequency over time. To solve this issue, a switch is used in the Vout calculation.
First, the first few instruments (0-2) were assigned to be waveform generating and the remaining (3-4) to be the beats. If instru variable is greater than 2, we enter the beat generating section.
In the beat generating section, the entries stored in waveform_lut must only repeat for 1 period then stop. To do this, the timer variable t is clipped at the maximum period length of a particular key tone.

## Communication

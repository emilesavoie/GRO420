#ifndef SONG_HPP
#define SONG_HPP

// Tempo in beats-per-minute:
#define TEMPO_BPM       120
// Length of a 1/16 note in ms:
#define TEMPO_16T_MS    ((60*250)/TEMPO_BPM)

// Note definition
// 
// 1/4  note -> noire
// 1/16 note -> double-croche
//
// Each note has a specific frequency (in Hz) and lasts a multiple of 1/16 notes
// defined by TEMPO_16T_MS (in ms).
// The tempo is defined by the length of a 1/4 note in beats per minute.
// A tempo of 60 means each 1/4 note lasts 1 second, meaning each 1/16 note lasts 250 ms.
struct Note
{
    float   freq;       // Frequency in Hz.
    int     duration;   // Duration in 1/16 notes.
};

// A song is simply defined as an array of note.
// It assumes notes are played in order (no polyphony).
    // Mélodie complète du thème principal de Zelda
Note song[] = {
   {349, 2},
    {349, 4},
    {466, 8},
    {349, 12},
    {466, 4},
    {466, 2},
    {523, 2},
    {587, 2},
    {622, 2},
    {698, 16},
    {698, 4},
    {698, 4},
    {698, 4},
    {740, 2},
    {831, 2},
    {932, 24},
    {1109, 8},
    {1047, 8},
    {880, 16},
    {698, 8},
    {740, 24},
    {932, 8},
    {880, 8},
    {698, 16},
    {698, 8},
    {740, 24},
    {932, 8},
    {880, 8},
    {698, 16},
    {587, 8},
    {622, 24},
    {740, 8},
    {698, 8},
    {554, 16},
    {466, 8},
    {523, 6},
    {587, 2},
    {659, 16},
    {784, 4},
    {698, 2},
    {349, 2},
    {349, 2},
    {349, 2},
    {349, 2},
    {349, 2},
    {349, 2},
    {349, 2},
    {349, 4},
    {349, 2}

};

    


// Note frequency reference
// C3	    130.81
// C#3/Db3 	138.59	
// D3	    146.83	
// D#3/Eb3 	155.56	
// E3	    164.81
// F3	    174.61
// F#3/Gb3 	185.00
// G3	    196.00	
// G#3/Ab3 	207.65
// A3	    220.00
// A#3/Bb3 	233.08	
// B3	    246.94	

#endif

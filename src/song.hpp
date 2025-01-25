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
Note song[] = {
    {392.0f, 4},   // G4 - Heroic opening
    {440.0f, 2},   // A4 - Quick ascent
    {493.9f, 4},   // B4 - Climactic peak
    
    // Main Theme Progression
    {523.3f, 6},   // C5 - Energetic main melody
    {493.9f, 2},   // B4 - Descending resolution
    {440.0f, 4},   // A4 - Lighter transition
    
    // Adventurous Sequence
    {392.0f, 3},   // G4 - Rhythmic base
    {349.2f, 3},   // F4 - Descending pattern
    {329.6f, 4},   // E4 - Lower resolution
    
    // Heroic Buildup
    {392.0f, 6},   // G4 - Triumphant return
    {440.0f, 4},   // A4 - Rising excitement
    {493.9f, 8},   // B4 - Final heroic flourish
    
    // Battle Motif
    {587.3f, 3},   // D5 - Sharp attack note
    {622.3f, 2},   // D#5 - Intense accent
    {659.3f, 4},   // E5 - Climactic battle tone
    
    // Resolution Sequence
    {493.9f, 5},   // B4 - Descending triumph
    {440.0f, 3},   // A4 - Gentle conclusion
    {392.0f, 6},   // G4 - Final peaceful note
    
    // Extended Exploration
    {330.0f, 4},   // E4 - Soft interlude
    {370.0f, 3},   // F#4 - Transitional tone
    {415.0f, 5},   // G#4 - Ascending passage
    {466.2f, 4},   // A#4 - Sharp accent
    {523.3f, 7},   // C5 - Triumphant return
    {494.0f, 3},   // B4 - Resolution
    {440.0f, 5}    // A4 - Final descent
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

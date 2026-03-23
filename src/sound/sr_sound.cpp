

// q: Does SDL sound buffer callback run in a separate thread?
// a: Yes, the SDL sound buffer callback runs in a separate thread.

// q: how to avoid audio glitches when loading large audio files?
// a: To avoid audio glitches when loading large audio files, load them asynchronously in a separate thread before playback.

// q: how to load async audio in SDL2?
// a: To load audio asynchronously in SDL2, you can use a separate thread to load the audio data using SDL_LoadWAV or similar functions, and then queue the audio data for playback once loading is complete.

// q: to avoid using multithreading, how to load audio without glitches in SDL2?



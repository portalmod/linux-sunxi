./up-soundcard
jackd -dalsa -D -Phw:1 -Chw:1 &
jack_connect system:capture_1 system:playback_1
jack_connect system:capture_1 system:playback_2
jack_connect system:capture_2 system:playback_2
jack_connect system:capture_2 system:playback_1

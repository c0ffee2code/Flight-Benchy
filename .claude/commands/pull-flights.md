Run `python pipelines/flight-runner/scripts/pull_flights.py` and report the result.

Pulls all new runs from the SD card and deletes them from the card after a successful byte-exact transfer.
Pass `--erase` to also delete ghost/failed runs that couldn't be transferred.
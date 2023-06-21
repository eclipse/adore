## Known Issues

During an initial build there is a significant amount of data that is pulled and 
cached from the internet. In order to lessen this burden the tool AptCacherNg is
used. This added complexity has a drawback if run on an unreliable network 
resulting in non-deterministic HTTP errors and corrupted apt packages.

AptCacherNg has significant benefits but can also cause problems, this section
will present a few possible solutions.

Resolution steps:
- Rerun the build again, it will continue at the previous failure point.
- Delete corrupted packages with the apt cacher web interface.
  1. navigate to http://127.0.0.1:3142/acng-report.html in your browser 
  2. Check the: "Validate by file name AND file directory (use with care)," and "then validate file contents through checksum (SLOW), also detecting corrupt files," check boxes.
  3. Then click the "Start Scan and/or Expiration" button.
  4. Then click "Check all" button. 
  5. Then click "Delete selected files" button followed by "Delete now" button and close the web browser.
  6. Rerun the make build command.
- Disable the apt cacher ng by clearing the DOCKER_CONFIG environmental variable:
```bash
DOCKER_CONFIG= make <target>
```

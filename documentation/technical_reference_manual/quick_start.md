# ADORe Quick Start
This is a quick start guide to getting up and running with ADORe with no fuss.

To setup and configure ADORe for a first run you can run the following setup script:
```bash
bash <(curl -sSL https://raw.githubusercontent.com/DLR-TS/adore_tools/master/tools/adore_setup.sh)
```
Follow any on-screen instructions when the ADORe setup script is running or you
can run the installation in non-interactive/unattended mode:
```bash
bash <(curl -sSL https://raw.githubusercontent.com/DLR-TS/adore_tools/master/tools/adore_setup.sh) --headless
```

This script will do the following:
 
 - Verify that your system meets the minimum requirements to run ADORe 
 - Install the system dependencies GNU Make and Docker
 - Clone ADORe to your home directory
 - Build ADORe core components

For a more nuanced and deeper look into getting ADORe set up please review the 
[Getting Started](setup/getting_started.md) guide.

> **⚠️  WARNING:**
> As a general rule you should never run shell scripts from untrusted sources. 

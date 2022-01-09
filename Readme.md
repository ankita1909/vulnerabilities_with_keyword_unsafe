This project contains implementation uart by using SCB5 peripheral with Rust. The data sent from the keyboard can be viewed on Tera Term/ Putty.
The psoc6_02.svd file has been updated to include SCB5 with base address 0x4065000 similar to SCB0.

Prerequisites:

- Install the required tools as described in [Embedded Rust Book](https://doc.rust-lang.org/beta/embedded-book/intro/install.html)
- Build this project using `cargo build`
- Upgrade Board KitProg to KitProg3
- Download TeraTerm/Putty.
- Clone the repo `ssh://git@bitbucket.cloudapps.infineon.com:7999/~kumarankita/psoc6-o2-pac.git`

- Install Cypress OpenOCD
    [Manual](https://www.cypress.com/file/463231/download)
    [Download](https://github.com/cypresssemiconductorco/openocd/releases)

TeraTerm Configuration:
    - Baudrate: 9600
    - Stop Bit: 1
    - Parity: None
    - Data: 8bit

For Debugiing/Running the project

- Clone the project and make sure that psoc6-02-pac folder has updated psoc6_02.svd file containing SCB5 needed for the serial communication
- Open the project with vscode.
- Run `cargo build` in the terminal window.
- Goto Run -> Start Debugging/ Run without debugging.

Start sending/receiving the data using TeraTerm.


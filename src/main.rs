#![no_std]
#![no_main]

use core::alloc;
use core::mem;
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use psoc6_01_pac as psoc;
mod uart;
//use byteorder::{ByteOrder, LittleEndian};


#[link_section=".cm0boot"]
#[no_mangle]
static CM0_BOOT: [u8; 6224]=*(include_bytes!("psoc6_02_cm0p_sleep.bin"));
const PATTERN: u32 = 0xdeadbeef;
static mut DATA: [u32; 2048] = [PATTERN; 2048]; //data is an array of 0xdeadbeef
static mut t:u32 = 0;
#[inline(never)]
fn fib(n: u32) -> u32 {

    // allocate and zero a 1KB of stack memory
    let _use_stack = [0u8; 2048];
    unsafe{
        t = t +1;
        hprintln!("Called fibonaci function {} time", t).unwrap();
    }

    if n < 2 {
        1
    } else {
        fib(n - 1) + fib(n - 2)
    }
}


#[entry]
fn main() -> ! {

    hprintln!("Hello,world!").unwrap();

    //let mut t = 0;
    //let mut incoming_data:[u8;70] = [0;70];


   loop{
    unsafe { DATA[0] = 10; }
    //let a = 0xaaaaaa;
   // let fib_func = fib(100);

//Dangling pointer
   let p;
   {
    let mut x: u8 = 1;
    p = x as *const i32;;
   }

    let c = p;
    unsafe{
    hprintln!("{}",*p);
    }

   let i: u32 = 1;
   let p_imm = &i ;
   let mut m: u32 = 2;
   // implicit coercion
   let p_mut: *mut u32 = &mut m;
   //raw pointer derefrencing, not allowed without unsafe
   unsafe {
       let ref_imm: &u32 = &*p_imm;
       hprintln!("{}",*ref_imm);
   }

   // null pointer

   let k = 0usize;
    unsafe {
        let l = k as *const i32;
        hprintln!("{}", *l);
    }

    //buffer overflow
    let x = 3;
    let y = &x;
    unsafe {
        let z = y as *const i32;
        let a = *z.offset(2); // i can access values at next addresses
        let mut ch = &mut z.offset(3);
        *ch = 4 as *const i32;
        hprintln!("{}", *z.offset(1)); 
        hprintln!("{}", *y); 
        
    } 
    //uninitialised value
    /*
    let x: i32;
    unsafe{
        hprintln!("{}", x);
    }
    */ // compile error

 /* buffer overflow using raw pointers */

    let a: [u8; 3] = [2,3,4];
    let prt: *const [u8;3] = &a;

    let big_ptr: *mut [u8;20] = prt as *mut [u8;20];

    unsafe {
      let mut new_array: &mut [u8;20] = &mut *big_ptr;
      new_array[10] = 10;
      new_array[2] = 20;
      new_array[0] = 11;
      new_array[19] = 15;
      hprintln!("Hi");
     }
    }
   


    }

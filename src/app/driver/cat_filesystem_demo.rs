// =======================================================================================
// File: cat_filesystem_demo.rs
//
// Copyright (c) 2025 Vanderbilt University
// Authors:
// - Tengyue Wu
// 
//
// Licensed to Vanderbilt University. All Rights Reserved.
// =======================================================================================

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

// 导入 CAT 模块
use crate::app::cat::{CAT24C32, CAT24C32Config, Error};
use crate::app::cat::{NANDFileSystem, FileSystemConfig, FileSystemInfo};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    
    // 配置 LED 用于状态指示
    let mut led = Output::new(p.P0_17, Level::Low, OutputDrive::Standard);
    
    // 配置 I2C (TWI)
    let config = twim::Config::default();
    let mut i2c = Twim::new(p.TWISPI0, p.P0_03, p.P0_04, config);
    
    // 创建 CAT24C32 实例
    let cat_config = CAT24C32Config::default();
    let mut cat = CAT24C32::new(&mut i2c, cat_config);
    
    info!("CAT24C32 File System Demo Started");
    
    // 检查设备连接
    match cat.ping().await {
        Ok(true) => {
            info!("CAT24C32 detected successfully");
            led.set_high();
            Timer::after(Duration::from_millis(1000)).await;
            led.set_low();
        }
        Ok(false) => {
            info!("CAT24C32 not detected");
            return;
        }
        Err(e) => {
            info!("Error detecting CAT24C32: {:?}", e);
            return;
        }
    }
    
    // 创建文件系统
    let fs_config = FileSystemConfig::default();
    let mut fs = NANDFileSystem::new(&mut cat, fs_config);
    
    // 初始化文件系统
    match fs.initialize().await {
        Ok(_) => {
            info!("File system initialized successfully");
            led.set_high();
            Timer::after(Duration::from_millis(500)).await;
            led.set_low();
        }
        Err(e) => {
            info!("Error initializing file system: {:?}", e);
            return;
        }
    }
    
    // 演示文件系统操作
    demo_file_operations(&mut fs, &mut led).await;
    
    info!("CAT24C32 file system demo completed successfully");
}

/// 演示文件系统操作
async fn demo_file_operations(fs: &mut NANDFileSystem, led: &mut Output) {
    info!("Starting file system operations demo...");
    
    // 1. 创建文件
    info!("Creating files...");
    
    // 创建文本文件
    let text_data = b"This is a text file created by nRF52832!";
    match fs.create_file("test.txt", 0x01, text_data).await {
        Ok(_) => {
            info!("Created test.txt successfully");
            led.set_high();
            Timer::after(Duration::from_millis(300)).await;
            led.set_low();
        }
        Err(e) => {
            info!("Error creating test.txt: {:?}", e);
        }
    }
    
    // 创建配置文件
    let config_data = b"{\"device\":\"nRF52832\",\"version\":\"1.0.0\",\"mode\":\"normal\"}";
    match fs.create_file("config.json", 0x02, config_data).await {
        Ok(_) => {
            info!("Created config.json successfully");
            led.set_high();
            Timer::after(Duration::from_millis(300)).await;
            led.set_low();
        }
        Err(e) => {
            info!("Error creating config.json: {:?}", e);
        }
    }
    
    // 创建二进制文件
    let binary_data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08];
    match fs.create_file("data.bin", 0x03, &binary_data).await {
        Ok(_) => {
            info!("Created data.bin successfully");
            led.set_high();
            Timer::after(Duration::from_millis(300)).await;
            led.set_low();
        }
        Err(e) => {
            info!("Error creating data.bin: {:?}", e);
        }
    }
    
    // 2. 列出文件
    info!("Listing files...");
    let files = fs.list_files();
    info!("Found {} files:", files.len());
    
    for file in &files {
        let name = core::str::from_utf8(&file.name).unwrap_or("Unknown");
        let name_clean = name.trim_matches('\0');
        info!("- {} (Type: 0x{:02X}, Size: {} bytes)", name_clean, file.file_type, file.size);
    }
    
    // 3. 读取文件
    info!("Reading files...");
    
    // 读取文本文件
    match fs.read_file("test.txt").await {
        Ok(data) => {
            let content = core::str::from_utf8(&data).unwrap_or("Invalid UTF-8");
            info!("test.txt content: {}", content);
            led.set_high();
            Timer::after(Duration::from_millis(500)).await;
            led.set_low();
        }
        Err(e) => {
            info!("Error reading test.txt: {:?}", e);
        }
    }
    
    // 读取配置文件
    match fs.read_file("config.json").await {
        Ok(data) => {
            let content = core::str::from_utf8(&data).unwrap_or("Invalid UTF-8");
            info!("config.json content: {}", content);
            led.set_high();
            Timer::after(Duration::from_millis(500)).await;
            led.set_low();
        }
        Err(e) => {
            info!("Error reading config.json: {:?}", e);
        }
    }
    
    // 读取二进制文件
    match fs.read_file("data.bin").await {
        Ok(data) => {
            info!("data.bin content: {:?}", data);
            led.set_high();
            Timer::after(Duration::from_millis(500)).await;
            led.set_low();
        }
        Err(e) => {
            info!("Error reading data.bin: {:?}", e);
        }
    }
    
    // 4. 获取文件系统信息
    let fs_info = fs.get_filesystem_info();
    info!("File system info:");
    info!("- Total files: {}", fs_info.total_files);
    info!("- Total size: {} bytes", fs_info.total_size);
    info!("- Free space: {} bytes", fs_info.free_space);
    
    // 5. 删除文件
    info!("Deleting test.txt...");
    match fs.delete_file("test.txt").await {
        Ok(_) => {
            info!("test.txt deleted successfully");
            led.set_high();
            Timer::after(Duration::from_millis(1000)).await;
            led.set_low();
        }
        Err(e) => {
            info!("Error deleting test.txt: {:?}", e);
        }
    }
    
    // 6. 再次列出文件
    info!("Listing files after deletion...");
    let files_after = fs.list_files();
    info!("Found {} files after deletion:", files_after.len());
    
    for file in &files_after {
        let name = core::str::from_utf8(&file.name).unwrap_or("Unknown");
        let name_clean = name.trim_matches('\0');
        info!("- {} (Type: 0x{:02X}, Size: {} bytes)", name_clean, file.file_type, file.size);
    }
    
    info!("File system operations demo completed");
} 
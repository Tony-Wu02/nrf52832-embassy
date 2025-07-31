// =======================================================================================
// File: cat24c32.rs
//
// Copyright (c) 2025 Vanderbilt University
// Authors:
// - Tengyue Wu
// 
//
// Licensed to Vanderbilt University. All Rights Reserved.
// =======================================================================================

use embassy_nrf::twim::Twim;
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::gpio::{Pin, Level, Output, OutputDrive};
use embassy_time::{Duration, Timer};
use embedded_hal::blocking::i2c::{Write, WriteRead};

/// CAT24C32 EEPROM 配置
pub struct CAT24C32Config {
    /// I2C 地址 (默认 0x50)
    pub address: u8,
    /// 页面大小 (默认 32 字节)
    pub page_size: u16,
    /// 总容量 (默认 4096 字节)
    pub capacity: u16,
    /// 写保护引脚
    pub wp_pin: Option<Pin>,
}

impl Default for CAT24C32Config {
    fn default() -> Self {
        Self {
            address: 0x50,
            page_size: 32,
            capacity: 4096,
            wp_pin: None,
        }
    }
}

/// CAT24C32 EEPROM 驱动
pub struct CAT24C32<'a> {
    i2c: &'a mut Twim<'a, TWISPI0>,
    config: CAT24C32Config,
}

impl<'a> CAT24C32<'a> {
    /// 创建新的 CAT24C32 实例
    pub fn new(i2c: &'a mut Twim<'a, TWISPI0>, config: CAT24C32Config) -> Self {
        Self { i2c, config }
    }

    /// 读取单个字节
    pub async fn read_byte(&mut self, address: u16) -> Result<u8, Error> {
        let addr_bytes = address.to_be_bytes();
        let mut buffer = [0u8; 1];
        
        self.i2c
            .write_read(self.config.address, &addr_bytes, &mut buffer)
            .await
            .map_err(|_| Error::I2CError)?;
        
        Ok(buffer[0])
    }

    /// 读取多个字节
    pub async fn read_bytes(&mut self, address: u16, data: &mut [u8]) -> Result<(), Error> {
        let addr_bytes = address.to_be_bytes();
        
        self.i2c
            .write_read(self.config.address, &addr_bytes, data)
            .await
            .map_err(|_| Error::I2CError)?;
        
        Ok(())
    }

    /// 写入单个字节
    pub async fn write_byte(&mut self, address: u16, data: u8) -> Result<(), Error> {
        let addr_bytes = address.to_be_bytes();
        let buffer = [addr_bytes[0], addr_bytes[1], data];
        
        self.i2c
            .write(self.config.address, &buffer)
            .await
            .map_err(|_| Error::I2CError)?;
        
        // 等待写入完成
        Timer::after(Duration::from_millis(5)).await;
        
        Ok(())
    }

    /// 写入页面数据
    pub async fn write_page(&mut self, address: u16, data: &[u8]) -> Result<(), Error> {
        if data.len() > self.config.page_size as usize {
            return Err(Error::PageSizeExceeded);
        }

        let addr_bytes = address.to_be_bytes();
        let mut buffer = vec![addr_bytes[0], addr_bytes[1]];
        buffer.extend_from_slice(data);
        
        self.i2c
            .write(self.config.address, &buffer)
            .await
            .map_err(|_| Error::I2CError)?;
        
        // 等待写入完成
        Timer::after(Duration::from_millis(10)).await;
        
        Ok(())
    }

    /// 擦除整个 EEPROM
    pub async fn erase_all(&mut self) -> Result<(), Error> {
        let zero_data = vec![0u8; self.config.page_size as usize];
        
        for page in 0..(self.config.capacity / self.config.page_size) {
            let address = page * self.config.page_size;
            self.write_page(address, &zero_data).await?;
        }
        
        Ok(())
    }

    /// 检查设备是否响应
    pub async fn ping(&mut self) -> Result<bool, Error> {
        let mut buffer = [0u8; 1];
        match self.i2c.write_read(self.config.address, &[], &mut buffer).await {
            Ok(_) => Ok(true),
            Err(_) => Ok(false),
        }
    }
}

/// NAND 文件格式支持
pub struct NANDFileFormat {
    /// 文件头标识
    pub header: [u8; 4],
    /// 文件大小
    pub file_size: u32,
    /// 文件类型
    pub file_type: u8,
    /// 校验和
    pub checksum: u16,
}

impl NANDFileFormat {
    /// 创建新的 NAND 文件格式
    pub fn new(file_type: u8) -> Self {
        Self {
            header: *b"NAND",
            file_size: 0,
            file_type,
            checksum: 0,
        }
    }

    /// 计算校验和
    pub fn calculate_checksum(&self, data: &[u8]) -> u16 {
        let mut checksum: u16 = 0;
        for &byte in data {
            checksum = checksum.wrapping_add(byte as u16);
        }
        checksum
    }

    /// 序列化文件头
    pub fn serialize_header(&self) -> Vec<u8> {
        let mut header = Vec::new();
        header.extend_from_slice(&self.header);
        header.extend_from_slice(&self.file_size.to_le_bytes());
        header.push(self.file_type);
        header.extend_from_slice(&self.checksum.to_le_bytes());
        header
    }

    /// 反序列化文件头
    pub fn deserialize_header(data: &[u8]) -> Result<Self, Error> {
        if data.len() < 12 {
            return Err(Error::InvalidHeader);
        }

        let header = [data[0], data[1], data[2], data[3]];
        if header != *b"NAND" {
            return Err(Error::InvalidHeader);
        }

        let file_size = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
        let file_type = data[8];
        let checksum = u16::from_le_bytes([data[9], data[10]]);

        Ok(Self {
            header,
            file_size,
            file_type,
            checksum,
        })
    }
}

/// 错误类型
#[derive(Debug, Clone, Copy)]
pub enum Error {
    I2CError,
    PageSizeExceeded,
    InvalidHeader,
    WriteError,
    ReadError,
    AddressOutOfRange,
} 
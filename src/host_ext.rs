pub(crate) trait OtgFsHostExt {
	fn hcintx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINT0;
	fn hccharx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0;
	fn hcintmskx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0;
	fn hctsizx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0;
}

impl OtgFsHostExt for stm32f4xx_hal::pac::OTG_FS_HOST {
	fn hcintx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINT0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCINT0 = &self.hcint0;
		unsafe { &*((ptr as usize).wrapping_add(i as usize * 0x20) as *const stm32f4xx_hal::stm32::otg_fs_host::HCINT0) }
	}
	fn hcintmskx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0 = &self.hcintmsk0;
		unsafe { &*((ptr as usize).wrapping_add(i as usize * 0x20) as *const stm32f4xx_hal::stm32::otg_fs_host::HCINTMSK0) }
	}
	fn hctsizx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0 = &self.hctsiz0;
		unsafe { &*((ptr as usize).wrapping_add(i as usize * 0x20) as *const stm32f4xx_hal::stm32::otg_fs_host::HCTSIZ0) }
	}
	fn hccharx(&self, i: u8) -> &stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0 {
		assert!(i < 8);
		let ptr: *const stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0 = &self.hcchar0;
		unsafe { &*((ptr as usize).wrapping_add(i as usize * 0x20) as *const stm32f4xx_hal::stm32::otg_fs_host::HCCHAR0) }
	}
}

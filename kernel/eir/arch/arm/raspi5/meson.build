raspi5_sources = files('raspi5.S', '../raspi4/raspi4.cpp')
objcopy = find_program('aarch64-managarm-objcopy')

exe = executable('eir-raspi5', eir_sources, raspi5_sources,
	include_directories : eir_includes,
	cpp_args : [eir_cpp_args, '-DRASPI5'],
	link_args : [eir_link_args, '-Wl,-T,' + meson.current_source_dir() + '/link.x'],
	dependencies : eir_dependencies,
	link_depends : files('link.x'),
	install : true,
)

custom_target('kernel8.img',
	input : exe,
	output : 'kernel8.img',
	build_by_default : true,
	command : [ objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@' ]
)

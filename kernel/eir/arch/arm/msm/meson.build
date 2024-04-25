msm_sources = files('msm.S', 'msm.cpp')
objcopy = find_program('aarch64-managarm-objcopy')

exe = executable('eir-msm', eir_sources, msm_sources,
	include_directories : eir_includes,
	cpp_args : [eir_cpp_args, '-DPIE'],
	link_args : [eir_link_args, '-Wl,-T,' + meson.current_source_dir() + '/link.x', '-static-pie'],
	dependencies : eir_dependencies,
	link_depends : files('link.x'),
	pie : true,
	install : true
)

custom_target('eir-msm.bin',
	input : exe,
	output : 'eir-msm.bin',
	build_by_default : true,
	command : [ objcopy, '-O', 'binary', '@INPUT@', '@OUTPUT@' ]
)

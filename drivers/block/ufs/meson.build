src = [
	'src/main.cpp',
	'src/controller.cpp',
	'src/lu.cpp',
]

executable('block-ufs', src,
	dependencies : [ libarch, hw_proto_dep, mbus_proto_dep, libblockfs_dep, core_dep, svrctl_proto_dep ],
	install : true
)

custom_target('block-ufs-server',
	command : [bakesvr, '-o', '@OUTPUT@', '@INPUT@'],
	output : 'block-ufs.bin',
	input : 'block-ufs.yml',
	install : true,
	install_dir : server
)

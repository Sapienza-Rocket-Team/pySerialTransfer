class CRC:
	def __init__(self, polynomial: int = 0x9B, crc_len: int = 8) -> None:
		self.poly      = polynomial & 0xFF
		self.crc_len   = crc_len
		self.table_len = pow(2, crc_len)
		self.cs_table  = [0 for _ in range(self.table_len)]

		self.generate_table()

	def generate_table(self) -> None:
		for i in range(len(self.cs_table)):
			curr = i

			for j in range(8):
				if (curr & 0x80) != 0:
					curr = ((curr << 1) & 0xFF) ^ self.poly
				else:
					curr <<= 1

			self.cs_table[i] = curr

	def print_table(self) -> None:
		buffer = ''
		for i in range(len(self.cs_table)):
			buffer += hex(self.cs_table[i]).upper().replace('X', 'x')

			if (i + 1) % 16:
				buffer += ' '
			else:
				buffer += '\n'
		print( buffer )

	def calculate(self, arr: list[str] | int, dist: int | None = None) -> int:
		crc = 0

		if isinstance( arr, list ):
			indicies = dist or len(arr)

			for i in range(indicies):
				try:
					nex_el = int(arr[i])
				except ValueError:
					nex_el = ord(arr[i])

				crc = self.cs_table[crc ^ nex_el]
		elif isinstance( arr, int ):
			crc = self.cs_table[arr]  # FIXME: ???
		else:
			raise TypeError( f'Invalid parameter `arr` type: expected `list[str]`, found `{type(arr)}`' )

		return crc


if __name__ == '__main__':
	def _main() -> None:
		crc = CRC()
		print(crc.print_table())
		print(' ')
		print(hex(crc.calculate(0x31)).upper().replace('X', 'x'))

	_main()

import pstats

stats = pstats.Stats('profile_results1.out')
stats.strip_dirs()
stats.sort_stats('cumtime')
stats.print_stats()

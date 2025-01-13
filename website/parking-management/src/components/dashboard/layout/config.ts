import type { NavItemConfig } from '@/types/nav';
import { paths } from '@/paths';

export const navItems = [
  { key: 'overview', title: 'Overview', href: paths.dashboard.overview, icon: 'chart-pie' },
  { key: 'customers', title: 'Parking Management', href: paths.dashboard.parking_management, icon: 'users' },
  { key: 'integrations', title: 'Car info', href: paths.dashboard.car_info, icon: 'plugs-connected' },
  { key: 'error', title: 'Trajectory Editor (In progress)', href: paths.errors.notFound, icon: 'x-square' },
  //{ key: 'settings', title: 'Settings', href: paths.dashboard.settings, icon: 'gear-six' },
  //{ key: 'account', title: 'Account', href: paths.dashboard.account, icon: 'user' },
] satisfies NavItemConfig[];

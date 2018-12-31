import numpy as np


class TimeStamp(object):
	"""docstring for TimeStamp"""
	def __init__(self, _path, ndof, _v_max, _a_max, _delta_t, _t_start=0):
		assert isinstance(_path, list)
		assert len(_v_max) == ndof
		assert len(_a_max) == ndof
		self.path = _path
		self.dof = ndof
		self.v_max = _v_max
		self.a_max = _a_max
		self.v_profile = {}
		self.a_profile = {}
		for i in range(ndof):
			self.v_profile[i] = []
			self.a_profile[i] = []
		self.delta_t = _delta_t
		self.t_start = _t_start

	def segmentTimeStamp(self, t0, q_a, q_b):
		# find ds_max
		ds_max = 1e5
		for i in range(self.dof):
			cand = self.v_max[i] / abs(q_b[i] - q_a[i])
			if cand < ds_max:
				ds_max = cand
		# find dds_max
		dds_max = 1e5
		for i in range(self.dof):
			cand = self.a_max[i] / abs(q_b[i] - q_a[i])
			if cand < dds_max:
				dds_max = cand

		v_profile = [0]
		a_profile = [dds_max]
		t = t0
		if ds_max >= np.sqrt(dds_max):
			ts = 1 / np.sqrt(dds_max)
			while t <= t0 + ts:
				v = v_profile[-1] + dds_max * self.delta_t
				v_profile.append(v)
				a_profile.append(dds_max) 
				t += self.delta_t
			while t < t0 + 2 * ts:
				v = v_profile[-1] - dds_max * self.delta_t
				v_profile.append(v)
				a_profile.append(-dds_max) 
				t += self.delta_t
		else:
			tau_0 = ds_max / dds_max
			tau_1 = 1. / ds_max - tau_0
			while t <= t0 + tau_0:
				v = v_profile[-1] + dds_max * self.delta_t
				v_profile.append(v)
				a_profile.append(dds_max) 
				t += self.delta_t
			while t <= t0 + tau_0 + tau_1:
				v_profile.append(ds_max)
				a_profile.append(0.)
				t += self.delta_t
			while t < t0 + 2 * tau_0 + tau_1: 
				v = v_profile[-1] - dds_max * self.delta_t
				v_profile.append(v)
				a_profile.append(-dds_max) 
				t += self.delta_t
		
		return v_profile, a_profile, t

	def pathTimeStamp(self):
		t0 = self.t_start
		for i in range(len(self.path) - 1):
			q_a = self.path[i]
			q_b = self.path[i + 1] 
			ds_prof, dds_prof, tf = self.segmentTimeStamp(t0, q_a, q_b)
			# update t0
			t0 = tf 
			# calculate v_prof, a_prof for each dof
			for j in range(self.dof):
				self.v_profile[j] += [ds * (q_b[j] - q_a[j]) for ds in ds_prof]
				self.a_profile[j] += [dds * (q_b[j] - q_a[j]) for dds in dds_prof]	
			
